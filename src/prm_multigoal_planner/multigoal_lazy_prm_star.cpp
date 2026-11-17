// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#include <prm_multigoal_planner/multigoal_lazy_prm_star.hpp>

#include <algorithm>
#include <chrono>
#include <set>
#include <tuple>
#include <unordered_set>
#include <vector>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/property_map/property_map.hpp>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Exception.h>

namespace prm_multigoal_planner
{

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
// Same value as ompl::magic::MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION (private to LazyPRM.cpp).
constexpr unsigned int kMinAddedSegmentsForLazyOptimization = 5;

template <class Vertex>
struct FoundGoal
{
  Vertex v;
};
struct NoImprovement
{
};
}  // namespace

MultigoalLazyPRMstar::MultigoalLazyPRMstar(const ob::SpaceInformationPtr& si) : og::LazyPRMstar(si)
{
  setName("MultigoalLazyPRMstar");
}

bool MultigoalLazyPRMstar::startGoalConnected() const
{
  for (const Vertex& s : startM_)
    for (const Vertex& g : goalM_)
      if (vertexComponentProperty_[s] == vertexComponentProperty_[g])
        return true;
  return false;
}

bool MultigoalLazyPRMstar::tryDirectConnections(ob::PathPtr& best)
{
  // (cost, start, goal) of the pairs whose straight line already satisfies the threshold
  std::vector<std::tuple<double, Vertex, Vertex>> pairs;
  for (const Vertex& s : startM_)
    for (const Vertex& g : goalM_)
    {
      const ob::Cost c = opt_->motionCost(stateProperty_[s], stateProperty_[g]);
      if (opt_->isSatisfied(c) && opt_->isCostBetterThan(c, bestCost_))
        pairs.emplace_back(c.value(), s, g);
    }
  std::sort(pairs.begin(), pairs.end(),
            [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });

  for (const auto& [cost, s, g] : pairs)
  {
    if (!si_->checkMotion(stateProperty_[s], stateProperty_[g]))
      continue;

    // Keep the (now validated) edge in the roadmap.
    auto existing = boost::lookup_edge(s, g, g_);
    Edge e;
    if (existing.second)
      e = existing.first;
    else
    {
      const Graph::edge_property_type properties{ ob::Cost(cost) };
      e = boost::add_edge(s, g, properties, g_).first;
      uniteComponents(s, g);
    }
    edgeValidityProperty_[e] = VALIDITY_TRUE;

    best = std::make_shared<og::PathGeometric>(si_, stateProperty_[s], stateProperty_[g]);
    bestCost_ = ob::Cost(cost);
    return true;
  }
  return false;
}

MultigoalLazyPRMstar::SearchResult MultigoalLazyPRMstar::lazyMultigoalSearch(const Vertex& start, ob::PathPtr& path)
{
  path.reset();

  std::vector<Vertex> goals;
  for (const Vertex& g : goalM_)
    if (vertexComponentProperty_[g] == vertexComponentProperty_[start])
      goals.push_back(g);
  if (goals.empty())
    return SearchResult::NONE;
  const std::unordered_set<Vertex> goal_set(goals.begin(), goals.end());

  // Vertices may have been removed: the index map must be 0 .. N-1 again.
  unsigned long int index = 0;
  boost::graph_traits<Graph>::vertex_iterator vi, vend;
  for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
    indexProperty_[*vi] = index;

  // Heuristic: distance lower bound to the closest goal (min of consistent heuristics: consistent).
  auto heuristic = [this, &goals](Vertex v) {
    ob::Cost h = opt_->infiniteCost();
    for (const Vertex& g : goals)
    {
      const ob::Cost c = costHeuristic(v, g);
      if (opt_->isCostBetterThan(c, h))
        h = c;
    }
    return h;
  };

  std::vector<ob::Cost> dist(boost::num_vertices(g_), opt_->infiniteCost());
  auto dist_map = boost::make_iterator_property_map(dist.begin(), indexProperty_);
  auto prev = boost::get(boost::vertex_predecessor, g_);

  // Stop at the first goal popped from the queue (= the closest goal in the lazy roadmap), or as soon as
  // the smallest f = g + h cannot beat the incumbent anymore.
  struct Visitor : public boost::default_astar_visitor
  {
    const std::unordered_set<Vertex>* goal_set;
    const std::vector<ob::Cost>* dist;
    const boost::property_map<Graph, boost::vertex_index_t>::type* index;
    const ob::OptimizationObjective* opt;
    const ob::Cost* incumbent;
    std::function<ob::Cost(Vertex)> h;

    void examine_vertex(Vertex u, const Graph&) const
    {
      const ob::Cost f = opt->combineCosts((*dist)[(*index)[u]], h(u));
      if (!opt->isCostBetterThan(f, *incumbent))
        throw NoImprovement();
      if (goal_set->count(u) > 0)
        throw FoundGoal<Vertex>{ u };
    }
  } visitor;
  visitor.goal_set = &goal_set;
  visitor.dist = &dist;
  visitor.index = &indexProperty_;
  visitor.opt = opt_.get();
  visitor.incumbent = &bestCost_;
  visitor.h = heuristic;

  Vertex goal = start;
  bool found = false;
  try
  {
    boost::astar_search(g_, start, heuristic,
                        boost::predecessor_map(prev)
                            .distance_map(dist_map)
                            .distance_compare(
                                [this](ob::Cost c1, ob::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                            .distance_combine([this](ob::Cost c1, ob::Cost c2) { return opt_->combineCosts(c1, c2); })
                            .distance_inf(opt_->infiniteCost())
                            .distance_zero(opt_->identityCost())
                            .visitor(visitor));
  }
  catch (const FoundGoal<Vertex>& fg)
  {
    goal = fg.v;
    found = true;
  }
  catch (const NoImprovement&)
  {
    return SearchResult::NONE;
  }
  if (!found || prev[goal] == goal)
    return SearchResult::NONE;

  // ---- Lazy validation (same policy as LazyPRM::constructSolution) ----
  // Vertices first: removing an invalid vertex removes more of the graph than removing an edge.
  std::vector<const ob::State*> states(1, stateProperty_[goal]);
  std::set<Vertex> milestones_to_remove;
  for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos])
  {
    const ob::State* st = stateProperty_[pos];
    unsigned int& vd = vertexValidityProperty_[pos];
    if ((vd & VALIDITY_TRUE) == 0 && si_->isValid(st))
      vd |= VALIDITY_TRUE;
    if ((vd & VALIDITY_TRUE) == 0)
      milestones_to_remove.insert(pos);
    if (milestones_to_remove.empty())
      states.push_back(st);
  }
  // The goal itself (valid: checked when the query was added, but it may be an old roadmap vertex).
  {
    unsigned int& vd = vertexValidityProperty_[goal];
    if ((vd & VALIDITY_TRUE) == 0 && si_->isValid(stateProperty_[goal]))
      vd |= VALIDITY_TRUE;
  }

  if (!milestones_to_remove.empty())
  {
    const unsigned long int comp = vertexComponentProperty_[start];
    std::set<Vertex> neighbors;
    for (const Vertex& m : milestones_to_remove)
    {
      boost::graph_traits<Graph>::adjacency_iterator nbh, last;
      for (boost::tie(nbh, last) = boost::adjacent_vertices(m, g_); nbh != last; ++nbh)
        if (milestones_to_remove.count(*nbh) == 0)
          neighbors.insert(*nbh);
      nn_->remove(m);
      si_->freeState(stateProperty_[m]);
      boost::clear_vertex(m, g_);
      boost::remove_vertex(m, g_);
    }
    for (const Vertex& n : neighbors)
      if (vertexComponentProperty_[n] == comp)
      {
        const unsigned long int new_component = componentCount_++;
        componentSize_[new_component] = 0;
        markComponent(n, new_component);
      }
    return SearchResult::RETRY;
  }

  states.push_back(stateProperty_[start]);

  // Then the edges, removing the first invalid one.
  auto prev_state = states.begin();
  auto state = prev_state + 1;
  Vertex prev_vertex = goal;
  Vertex pos = prev[goal];
  do
  {
    const Edge e = boost::lookup_edge(pos, prev_vertex, g_).first;
    unsigned int& evd = edgeValidityProperty_[e];
    if ((evd & VALIDITY_TRUE) == 0 && si_->checkMotion(*state, *prev_state))
      evd |= VALIDITY_TRUE;
    if ((evd & VALIDITY_TRUE) == 0)
    {
      boost::remove_edge(e, g_);
      const unsigned long int new_component = componentCount_++;
      componentSize_[new_component] = 0;
      markComponent(pos, new_component);
      return SearchResult::RETRY;
    }
    prev_state = state;
    ++state;
    prev_vertex = pos;
    pos = prev[pos];
  } while (prev_vertex != pos);

  auto p = std::make_shared<og::PathGeometric>(si_);
  for (auto st = states.rbegin(); st != states.rend(); ++st)
    p->append(*st);
  path = p;
  return SearchResult::FOUND;
}

bool MultigoalLazyPRMstar::improveSolution(ob::PathPtr& best, const ob::PlannerTerminationCondition& ptc)
{
  for (const Vertex& s : startM_)
  {
    while (!ptc)
    {
      ob::PathPtr path;
      const auto t0 = std::chrono::steady_clock::now();
      const SearchResult r = lazyMultigoalSearch(s, path);
      stats_.search_time += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      ++stats_.searches;
      stats_.retries += (r == SearchResult::RETRY);
      if (r == SearchResult::NONE)
        break;
      if (r == SearchResult::RETRY)
        continue;

      const ob::Cost c = path->cost(opt_);
      if (opt_->isCostBetterThan(c, bestCost_))
      {
        best = path;
        bestCost_ = c;
      }
      break;
    }
    if (best && opt_->isSatisfied(bestCost_))
      return true;
  }
  return best && opt_->isSatisfied(bestCost_);
}

ob::PlannerStatus MultigoalLazyPRMstar::solve(const ob::PlannerTerminationCondition& ptc)
{
  checkValidity();
  query_iterations_ = 0;
  query_satisfied_ = false;
  stats_ = QueryStats();

  auto* goal = dynamic_cast<ob::GoalSampleableRegion*>(pdef_->getGoal().get());
  if (goal == nullptr)
  {
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  // New cost threshold for every query (LazyPRM::setup() reads the objective only once).
  if (pdef_->hasOptimizationObjective())
    opt_ = pdef_->getOptimizationObjective();

  while (const ob::State* st = pis_.nextStart())
    startM_.push_back(addMilestone(si_->cloneState(st)));
  if (startM_.empty())
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ob::PlannerStatus::INVALID_START;
  }

  // All the goal states, not just the first one as in LazyPRM.
  while (const ob::State* st = pis_.nextGoal())
    goalM_.push_back(addMilestone(si_->cloneState(st)));
  if (goalM_.empty())
  {
    OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
    return ob::PlannerStatus::INVALID_GOAL;
  }

  // Query vertices passed the validity check of PlannerInputStates.
  for (const Vertex& v : startM_)
    vertexValidityProperty_[v] |= VALIDITY_TRUE;
  for (const Vertex& v : goalM_)
    vertexValidityProperty_[v] |= VALIDITY_TRUE;

  OMPL_DEBUG("%s: query with %zu start(s), %zu goal(s), %lu milestone(s) in the roadmap", getName().c_str(),
             startM_.size(), goalM_.size(), milestoneCount());

  bestCost_ = opt_->infiniteCost();
  ob::PathPtr best;

  bool satisfied = false;
  if (try_direct_connection_)
    satisfied = tryDirectConnections(best);

  // The persistent roadmap may already contain a good enough path.
  if (!satisfied && startGoalConnected())
    satisfied = improveSolution(best, ptc);

  // Informed sampler of this query (its hyperspheroids depend on the starts and goals).
  ob::InformedSamplerPtr informed;
  if (informed_ratio_ > 0.0)
  {
    try
    {
      informed = opt_->allocInformedStateSampler(pdef_, 100u);
    }
    catch (const ompl::Exception& e)
    {
      OMPL_WARN("%s: informed sampling disabled: %s", getName().c_str(), e.what());
    }
  }

  ob::State* work_state = si_->allocState();
  unsigned long segments_since_last_search = 0;
  while (!satisfied && !ptc)
  {
    ++iterations_;
    ++query_iterations_;
    // Informed sample once a solution exists; uniform otherwise, or if rejection sampling failed
    // (informed set mostly outside the joint bounds).
    bool informed_sample = false;
    if (best && informed && rng_.uniform01() < informed_ratio_)
      informed_sample = informed->sampleUniform(work_state, bestCost_);
    if (informed_sample)
      ++stats_.informed_samples;
    else
      sampler_->sampleUniform(work_state);
    const Vertex added = addMilestone(si_->cloneState(work_state));

    if (!startGoalConnected())
      continue;

    if (best)
    {
      // Optimizing: search again only after a few new vertices in a start component.
      bool in_start_component = false;
      for (const Vertex& s : startM_)
        in_start_component |= (vertexComponentProperty_[s] == vertexComponentProperty_[added]);
      if (!in_start_component)
        continue;
      const unsigned long period = std::max<unsigned long>(
          kMinAddedSegmentsForLazyOptimization,
          static_cast<unsigned long>(search_growth_ratio_ * static_cast<double>(milestoneCount())));
      if (++segments_since_last_search < period)
        continue;
      segments_since_last_search = 0;
    }

    satisfied = improveSolution(best, ptc);
  }
  si_->freeState(work_state);

  query_satisfied_ = satisfied;
  OMPL_DEBUG("%s: %lu new sample(s), %lu milestone(s), best cost %.4f%s", getName().c_str(), query_iterations_,
             milestoneCount(), bestCost_.value(), satisfied ? " (threshold satisfied)" : "");

  if (best)
  {
    // PathGeometric owns copies of its states, so `best` does not alias the roadmap.
    ob::PlannerSolution psol(best);
    psol.setPlannerName(getName());
    psol.setOptimized(opt_, bestCost_, satisfied);
    pdef_->addSolutionPath(psol);
    return ob::PlannerStatus::EXACT_SOLUTION;
  }
  return ob::PlannerStatus::TIMEOUT;
}

}  // namespace prm_multigoal_planner
