# prm_multigoal_planner

Action server for `parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan` (same interface as
`parallel_multigoal_planner`). It plans with a **persistent multi-goal Lazy PRM\*** built on OMPL.

- The robot model comes from `robot_description` / `robot_description_semantic`.
- The planning scene comes from `/get_planning_scene` **before every request**.
- The roadmap of a planning group is **kept between requests** and rebuilt only when the scene
  changes (see [Scene change detection](#scene-change-detection)).
- Planning stops as soon as `cost < (1 + utopia_tolerance) * utopia`, where
  `utopia = min_i ||goal_i - start||` (Euclidean distance in joint space). If that never happens, it
  returns the best path found within the time budget.

```bash
ros2 launch prm_multigoal_planner prm_multigoal_planner.launch.py ik_solver_ns:=ur_ik_solver
# to use it in place of parallel_multigoal_planner (same action name):
ros2 launch prm_multigoal_planner prm_multigoal_planner.launch.py action_name:=parallel_multigoal_plan
# force a rebuild of all the roadmaps at the next request:
ros2 service call /prm_multigoal_planner/clear_roadmap std_srvs/srv/Trigger
```

## Request flow

1. The planner takes the group's lock, so requests for the same group run one at a time because
   they share the roadmap.
2. It fetches the scene and computes its hash. If the hash differs from the one the roadmap was
   built on, it creates a new `PlanningScene`, validity checker and empty roadmap.
3. The start comes from `start_state`. If `start_state` is empty, the scene's current state is used.
   The start is checked against the joint bounds and for collisions.
4. Goals: Cartesian elements go through `ik_solver` (`get_task_redundant_ik_array`, all solutions),
   and joint elements are used as they are. Candidates out of bounds or in collision are dropped
   before the utopia is computed.
5. `MultigoalLazyPRMstar::solve()`:
   - tries the straight lines start→goal that already satisfy the threshold, cheapest first;
   - searches the existing roadmap (lazy A\* towards the whole goal set);
   - grows the roadmap and searches again every `max(5, search_growth_ratio * vertices)` new
     vertices in the start component, until the threshold, the time limit, or a cancel. Once a
     solution exists, a fraction `informed_sampling_ratio` of the samples comes from the informed
     set (see [Informed sampling](#informed-sampling)).
6. Optionally runs OMPL `PathSimplifier`. This only shortens the returned path; the roadmap is not
   changed.

## MultigoalLazyPRMstar

This is a subclass of `ompl::geometric::LazyPRMstar` with its own `solve()`. Stock `LazyPRM::solve()`
does not fit here for three reasons:

- it adds only one goal state per query;
- it optimizes towards a single start/goal pair;
- it reads the cost threshold only once, in `setup()`.

The subclass:

- adds **all** goal states to the roadmap;
- runs one A\* per search from the start towards the goal set. The heuristic is the distance to the
  closest goal, and the search is pruned as soon as `g + h` cannot beat the incumbent. The first goal
  popped from the queue is therefore the best goal in the current roadmap;
- validates the path lazily, with the same policy as `LazyPRM::constructSolution()`: invalid vertices
  and edges are removed and the search is repeated. Every vertex/edge is collision checked at most
  once while the roadmap lives;
- re-reads the objective (and so the threshold) from the problem definition at every query.

Start and goal states stay in the roadmap as ordinary vertices.

### Informed sampling

Uniform samples in the joint box rarely fall where they could shorten the current path: in 6D the
box of a UR is ~10^6 rad^6, while the set of states that can improve a path of cost `c` is

```
X_inf = { x : ||x - start|| + min_i ||x - goal_i|| < c }
```

the union of one prolate hyperspheroid per start/goal pair (foci on start and goal). With `c` close
to the utopia it is a tiny fraction of the box.

After the first solution, each new sample is drawn from `X_inf` with probability
`informed_sampling_ratio` (default 0.7), using OMPL `PathLengthDirectInfSampler` (direct sampling,
the sampler of Informed RRT\* / BIT\*) with the current best cost. The rest stay uniform, to keep
exploring other homotopy classes and to keep the persistent roadmap useful for later queries. When
the hyperspheroid lies mostly outside the joint bounds, the rejection sampling may fail; that sample
is then drawn uniformly.

Informed samples stay in the roadmap. They are not pruned: the A\* already ignores vertices with
`g + h >= c`, and they can serve later queries.

## Scene change detection

`planningSceneHash()` is a 64-bit hash (FNV-1a) of the serialized `PlanningScene` after these
normalizations:

- **ignored**: name, object colors, header stamps, and the positions of the group's own joints
  (they are the planning variables);
- **included**: world objects and octomap, attached objects, ACM, link padding/scale, fixed frame
  transforms, and the positions of all the other joints (gripper, other arms), quantized to
  `scene_joint_tolerance`.

Collision objects and transforms are sorted by id first.

A live octomap, or any object that moves, changes the hash at every request, so the roadmap is
rebuilt every time.

## Collision checking

`MoveItStateValidityChecker` calls `PlanningScene::isStateColliding(state, group)`. It considers the
links moved by the group and the objects attached to them, checked against the rest of the robot and
the world, with the scene's ACM. Joints outside the group keep their values from the scene, not from
`start_state`.

Edges are checked every `collision_check_resolution` rad (OMPL `DiscreteMotionValidator`). A path
that grazes an obstacle may touch it between two checks. `collision_padding` [m] adds a safety margin:
it is added to the scene's padding of the links moved by the group, on the planner's copy of the scene
only (move_group is not changed).

- MoveIt applies link padding to robot-world checks only; self-collisions are unpadded.
- Start and goals are checked with the padding too: a state closer than the padding to an obstacle
  could not be connected anyway, since the first points checked along its edges would be in
  collision as well. Such a start gives `START_STATE_IN_COLLISION`, such goals are dropped.
- Links that are always close to a fixed object (e.g. the shoulder over the mounting plate) can make
  every state invalid with a large padding: keep it small (a few mm) or allow those pairs in the ACM.
- The padding is read when the roadmap is built; changing it rebuilds the roadmap at the next request.

With `collision_check_resolution` 0.02 rad the UR10e links move about 1 cm between two checks, so
3-5 mm covers the grazing cases seen in the tests.

## Parameters

See `config/prm_multigoal_planner.yaml`. Parameters fall into three groups:

- read at startup: `action_name`, `ik_solver_ns`, `planning_scene_service`, `service_timeout_s`;
- read when a roadmap is (re)built: `collision_check_resolution`, `scene_joint_tolerance`,
  `collision_padding` (a change of `collision_padding` triggers a rebuild);
- read at every request: everything else.

## Error codes

| Case | `MoveItErrorCodes` |
|---|---|
| unknown group / group with multi-variable joints | `INVALID_GROUP_NAME` |
| start missing joints or out of bounds | `INVALID_ROBOT_STATE` |
| start in collision | `START_STATE_IN_COLLISION` |
| malformed goal constraints / IK service error | `INVALID_GOAL_CONSTRAINTS` |
| no IK solution at all | `NO_IK_SOLUTION` |
| every candidate goal out of bounds or in collision | `INVALID_GOAL_CONSTRAINTS` (as parallel_multigoal_planner) |
| no path within the time budget | `PLANNING_FAILED` |
| canceled | `PREEMPTED` |

## Limitations

- Only groups made of single-variable joints are supported. Continuous joints use MoveIt's `[-pi, pi]`
  bounds with no wrap-around.
- Requests for the same group are serialized.
- The roadmap is rebuilt when it exceeds `max_roadmap_vertices`.

## Testing on a UR10e (mock hardware)

`test/ur10e/` contains a scenario file for `parallel_multigoal_planner_test/scripts/multigoal_planner_test.py`.
It plans for group `ur_manipulator` with frames `base_link`/`tool0`, and uses obstacles placed to
block the straight lines. The folder also holds the matching `ik_solver` configuration. Run each
command in its own terminal:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=127.0.0.1 \
    use_mock_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
    # Humble: use_fake_hardware:=true
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=false
ros2 launch ik_solver ik_solver.launch.py config:=test/ur10e/ur10e_ik.yaml
ros2 run prm_multigoal_planner prm_multigoal_planner_node --ros-args \
    -p action_name:=parallel_multigoal_plan -p ik_solver_ns:=ur_ik_solver
python multigoal_planner_test.py --ros-args -p scenarios_file:=test/ur10e/ur10e_scenarios.yaml \
    -p group_name:=ur_manipulator -p base_frame:=base_link -p tool_frame:=tool0 -p repetitions:=3
```

Results: 60/60 passed (20 scenarios x 3) in the RoboStack workspaces for Jazzy (OMPL 2.0), Humble
(OMPL 1.6) and Lyrical (OMPL 2.0).

Note: MoveIt uses the URDF `safety_controller` soft limits, so on a UR10e the elbow bounds are
±2.99 rad, not ±π.
