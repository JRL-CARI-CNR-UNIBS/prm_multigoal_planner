#pragma once

#include <algorithm>

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/util/RandomNumbers.h>

namespace prm_multigoal_planner
{

/**
 * @brief Lazy PRM* with a persistent roadmap and a multi-goal query.
 *
 * Differences from ompl::geometric::LazyPRM::solve():
 *  - all the goal states of the (GoalSampleableRegion) goal are added to the roadmap, not just one;
 *  - each time the roadmap may contain a better solution, ONE lazy A* search runs from the start
 *    towards the whole goal set (heuristic: distance to the closest goal, pruned on the incumbent
 *    cost), so the result is the best path to the best goal;
 *  - the optimization objective is re-read from the problem definition at every solve(), so that
 *    a new cost threshold (early termination) can be set for every query;
 *  - optionally, before growing the roadmap, straight-line connections start->goal that would
 *    already satisfy the cost threshold are tried (cheapest first);
 *  - after the first solution, a fraction of the new samples is drawn from the informed set
 *    { x : ||x - start|| + min_i ||x - goal_i|| < best cost } (OMPL PathLengthDirectInfSampler:
 *    union of one prolate hyperspheroid per start/goal pair). The informed samples stay in the
 *    persistent roadmap like the others.
 *
 * The roadmap (vertices, edges, and the cached validity of the vertices/edges checked so far) is
 * kept across queries: call clearQuery() (or setProblemDefinition()) between queries, and clear()
 * only when the validity checker changes meaning (e.g. the planning scene changed).
 * Start and goal states of a query stay in the roadmap as ordinary milestones.
 *
 * All the problem definitions given to the same instance must use objectives that compute the
 * same motion cost (e.g. always PathLengthOptimizationObjective), since edge weights are cached.
 */
class MultigoalLazyPRMstar : public ompl::geometric::LazyPRMstar
{
public:
  explicit MultigoalLazyPRMstar(const ompl::base::SpaceInformationPtr& si);

  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc) override;

  /// Try straight-line start->goal connections that already satisfy the cost threshold.
  void setTryDirectConnection(bool flag)
  {
    try_direct_connection_ = flag;
  }

  /**
   * @brief While optimizing, search again after max(5, ratio * roadmap size) new vertices in the start
   * component (each search costs O(roadmap size), so a fixed period makes big roadmaps sample slowly).
   * 0 = after every 5 vertices, as LazyPRM.
   */
  void setSearchGrowthRatio(double ratio)
  {
    search_growth_ratio_ = ratio;
  }

  /**
   * @brief Fraction [0, 1] of the samples drawn from the informed set once a solution exists
   * (0 = uniform sampling only). Requires a path-length objective on a RealVectorStateSpace;
   * otherwise sampling stays uniform.
   */
  void setInformedSamplingRatio(double ratio)
  {
    informed_ratio_ = std::clamp(ratio, 0.0, 1.0);
  }

  /// Cost of the best solution found in the current/last query (infinite if none).
  double bestCostValue() const
  {
    return bestCost_.value();
  }

  /// Roadmap samples added in the current/last query.
  unsigned long queryIterations() const
  {
    return query_iterations_;
  }

  /// Statistics of the current/last query.
  struct QueryStats
  {
    unsigned long searches{ 0 };      ///< lazy A* searches
    unsigned long retries{ 0 };       ///< searches that found invalid vertices/edges
    double search_time{ 0.0 };        ///< [s] spent in searches + lazy validation
    unsigned long informed_samples{ 0 };  ///< samples drawn from the informed set
  };
  const QueryStats& queryStats() const
  {
    return stats_;
  }

  /// Whether the last query ended because the cost threshold was satisfied.
  bool querySatisfied() const
  {
    return query_satisfied_;
  }

private:
  enum class SearchResult
  {
    FOUND,  ///< valid path to the closest goal, cheaper than the incumbent
    RETRY,  ///< invalid vertices/edge found and removed: search again
    NONE    ///< no goal reachable with a cost better than the incumbent
  };

  /**
   * @brief One lazy step: A* from @p start to the closest goal of its component (heuristic: distance to
   * the closest goal; pruned as soon as g + h cannot beat bestCost_), then validation of the found path
   * like LazyPRM::constructSolution() (invalid vertices/edge are removed from the roadmap).
   */
  SearchResult lazyMultigoalSearch(const Vertex& start, ompl::base::PathPtr& path);

  /**
   * @brief Repeat lazyMultigoalSearch() for every start until a valid improving path is found or none
   * exists in the current roadmap. Updates bestCost_ / best.
   * @return true if the best cost satisfies the objective threshold.
   */
  bool improveSolution(ompl::base::PathPtr& best, const ompl::base::PlannerTerminationCondition& ptc);

  /// Try the straight lines start->goal whose cost already satisfies the threshold (cheapest first).
  bool tryDirectConnections(ompl::base::PathPtr& best);

  /// True if at least one start shares its connected component with at least one goal.
  bool startGoalConnected() const;

  bool try_direct_connection_{ true };
  double search_growth_ratio_{ 0.01 };
  double informed_ratio_{ 0.0 };
  ompl::RNG rng_;
  unsigned long query_iterations_{ 0 };
  bool query_satisfied_{ false };
  QueryStats stats_;
};

}  // namespace prm_multigoal_planner
