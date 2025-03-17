^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package footstep_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2018-04-19)
------------------
* refacotoring to release on kinetic
* added documents related to footstep_planner packages
* added information new maintainer
* fixed CMakeLists.txt and package.xml files for dependency
* fixed footstep_planner node crashes when map changesth
* Contributors: Aleksey Titov, Kayman, Pyo

0.4.1 (2016-09-05)
------------------
* Added include files for footstep_planner and gridmap_2d for install.
* uses double 2.0 and makes style locally consistent
* catkinize the stack
* Add service to (re)plan between feet as start and goal.
* Requires newest humanoid_msgs.
* Merge plan and replan functions
* RViz launch file / config Groovy
* mark heuristic as expired when map is updated
* removed code in the destructor that gets already cleaned up in the super class
* use pre-increment instead of post-increment (faster + post-increment for iterators not always optimizable)
* fixed sbpl exception catch
* introduced new functionality to totally reset the planning environment
* storing environment params in new struct -- get rid of unnecessary member variables
* more detailed information when sbpl planning was successful but only the old path is returned
* wait 0.5 sec before updating the start (robot) pose
* bug fix in path cost heuristic
* make footsteps smaller; +  std::max
* handle receiving of foot transformations differently
* implemented the polygon-intersection-check for the navigation as well; not used so far due to some usability issues
* bug resolved: free the execution log when planning failed; use ros::Time(0) instead of ros::Time::now() since when using the latter sometimes the tf cannot be received (now sometimes an outdated step is received but this works better anyhow)
* bug resulting in segfaulting thread now (probably) resolved
* replaced old reachable check by a new more smoother grid based approach
* small fix in the replanning logic
* enforce planning from scratch when necessary (e.g. goal changed when using backward planning)
* added assertion to enforce correct heuristic calculation
* applied ROS code style
* reorganized the way FootstepNavigation invokes the planner and starts the execution; changed FootstepPlanner::updateMap

* Contributors: Armin Hornung, Johannes Garimort, Pramuditha Aravinda, Vincent Rabaud, enriquefernandez

0.4.0 (2013-01-10)
------------------
* Updated stack.xml & CMakeLists, new version 0.4.0 to release
* adjusted robot translation in z dir
* footstep execution now aborted when map changes + replanning and new execution is started; again use ALL calculated steps when extracting the path
* Changes in R* launch file params
* using only the new launch file from pkg nao_driver
* Fixed odometry lookup error (wrong frame)
* Indenting / code style according to ROS style
* fixed SBPL in CMakeListe for custom-installed location
* fixed out of map check in R*
* cleanup of R* code
* manifest/CMakeLists now compatible with fuerte and electric compilation, includes should work better now
* integrated the new footstep set
* fix: spelling error (don't forget to update nao_driver and humanoid_nav_msgs)
* navigation parameters in extra config file; using new launch file in nao_driver
* handle situations where both start feet are part of the calculated solution
* method rearrangement in the navigation
* got rid of get_footstep in helper
* added test to check whether all footsteps can be performed by the NAO robot; adjusted invalid footstep
* now able to switch between protective and fast footstep execution via the 'protective_execution' param + various smaller changes
* Finished fluid footstep execution for the NAO; has still to be tested for simulation/real robot
* added footstep_planner_walls to read in separate wall map (larger clearing)
* expanded states is now a hash map on x,y => vis.better in RVIZ, only one expanded state per (x,y)
* heuristic_scale parameter to increase under-estimating heuristics
* GetSuccsTo for improved R* functionality
* nao_path_follower now accepts paths
* Fixed PathCostHeuristic for non-square maps
* reaching right goal foot faster, only left goal state absorbs now.
* also show expanded states when failing; R* optimized (exact ==)
* PathCostHeuristic is now inflated by foot incircle

* Contributors: Armin Hornung, Daniel Maier, Johannes Garimort

0.3.1 (2012-06-15)
------------------
* preparing for fuerte release and sbpl package
* replanning based on old planning info in cases of a changed map disabled for now (instead: complete new planning in such a case)
* approx. comparison for R*
* footstep-feedback synchronization now handled in the action client (in FootstepNavigation.cpp)
* cost calculation directly based on the discrete planning states
* during the planning the reachability check is now based on the discrete position and orientation of the state (planning speed up)
* temp. simplification of the path cost heuristic calculation
* temp simplification of heuristic calculation
* code cleanup
* updated RViz config, launch file for R*
* slightly expanded large footstep set

* Contributors: Armin Hornung, Johannes Garimort

0.3.0 (2012-05-29)
------------------
* changed default planning params: 1cm res, PathCostHeuristic
* synchronized with footstep action server
* PathCostHeuristic fixed
* added fake localization launch file
* documentation update
* got rid of FootstepPlannerEnvironment::setUp - this is now done in the resp. updateGoal/updateStart-method (this makes it possible to reduce FootstepPlanner::run to its actual functionality)
* Removed ANAPlanner (testing, requires newer SBPL)
* Functions (stubs) added for R*
* changed names of the robot feet (according to ROS standards); integrate actionlib to perform footsteps
* corrected foot box for the nao robot
* corrected footsteps (w.r.t the new step model and the old footstep sets)
* footstep navigation debug and footstep execution debug update
* some bugs affecting footstep execution fixed
* footstep execution now completely discrete
* new function for lengths (to distinct between grid cell discretization and length discretization; footstep calculation fully based on discrete footsteps (can be further optimized by calculating reverse footstep on the fly)
* update of start pose in own function: used before planning and replanning
* bugfix: set gready footstep
* map callback subscription
* add missing goal pose callback subscription
* footstep robot navigation integration finished
* replaced double-ptr returns with references in get_footstep() / getFootstep()
* Added execution functions from old footstep planner code to FootstepNavigation
* FootstepNavigation stub added

* Contributors: Armin Hornung, Johannes Garimort

0.2.0 (2011-11-28)
------------------
* automatic replanning after map change
* reset planner when to many states changed after map change
* 2nd changed sample map added
* inserted stubs for R* env. functions.
* using old information in replanning after map change
* fixed invalid states appearing on border of map during expansion
* fixed corner cases for angles heuristic
* added foundation for map change detection
* get rid of last continuous planning state variable
* orientation cost
* fix on path cost heuristic and the recalculation w.r.t correct search direction
* path cost heuristic implementation finished
* footstep_planner now uses SBPL instead of D* lite, enabling ARA* and AD* for search with anytime capabilities
* Moved the PlanFootsteps service into humanoid_nav_msgs

* Contributors: Armin Hornung, Johannes Garimort

0.1.0 (2011-05-03)
------------------
* more maps added, adjusted launch files
* Added call to clear previous visualization
* Display of planning time in Dstar
* added Python node to plan from the command line
* added "PlanFootstep" service to FootstepPlanner
* added override for default marker namespace
* method to initiate heuristic values update from outside
* documentation
* private namespace for vis msgs
* A* heuristic implemented and integrated

* enhancements:

  * cleanup of param files
  * State, Footstep, Dstar classes separated
  * Got rid of rounding => huge speedup in planner
  * Better results with subgoal_distance=0.2
  * added Heuristic path visualization
  * helper functions inlined
  * replaced constants with step constants
  * access to Dstar path properties in FootstepPlanner
  * getPathCosts, getNumExpandedStates, getNumFootsteps added to receive planning information

* fixes:

  * fixed heuristic update after goal update in Dstar
  * fixed foot width for Astar heuristic

* Contributors: Armin Hornung, Johannes Garimort
