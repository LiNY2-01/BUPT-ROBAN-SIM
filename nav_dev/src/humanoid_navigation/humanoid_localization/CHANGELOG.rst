^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package humanoid_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2018-04-19)
------------------
* refacotoring to release on kinetic
* added documents related to humanoid_localization packages
* added information new maintainer
* fixed CMakeLists.txt and package.xml files for dependency
* resolved warnings
* Contributors: Kayman, Pyo

0.4.1 (2016-09-05)
------------------
* catkinize the stack
* get code to compile under Hydro
* removes some trailing blanks and semicolon
* added localization Service to HumanoidLocalization (untested)
* Localization updates are now done based on the distance to the last localized pose (not summed distance)
* Homogeneous subsampling of laser points (PCL voxel grid) instead of angular
* Uniform sampling for depth camera data
* Motion model and noise can now be calibrated with a variance matrix. Some parameters are no longer used, please adjust accordingly:
  motion_noise/x, motion_noise/y, motion_noise/yaw, num_sensor_beams
* Preventing TF_OLD_DATA Warning when init from RVIZ
* refactoring/cleanup
* Added Timer to Localization and timerCallback now republishes latest tf transform
* added parameterization for odometry calibration in MotionModel
* Proper floor offset if z is constrained to supporting surface by kinematics / odometry
* More loops parallelized (motion model)

* fixes:

  * Fixed invalid conversion from 'octomap::AbstractOcTree*' to 'octomap::OcTree*'
  * Add dependency to cmake_modules for FindEigen.cmake
  * Fixed duplicate library export
  * Fixed linking endpointmodel to humanoidlocalization in case dynamicEDT3D is available
  * Fix missing OpenMP compilation flags (from catkinization)
  * Fixing error with constrainMotion flags and optimized-away assertion
  * Fixed point cloud callback crashing when ground filtering and no ground visible
  * Bugfix for OctoMap in Groovy
  * Height lookup for endpoint model fixed
  * Fixed raycasting height integration on non-zero ground and raycast max_range
  * bug fix: removed (unnecessary) feature in HumanoidLocalization
  * Fixed compilation with ROS groovy (tf types)
  * fixed localization for point clouds

* Contributors: Armin Hornung, Daniel Maier, Pramuditha Aravinda, Stefan Osswald, Vincent Rabaud, enriquefernandez

0.4.0 (2013-01-10)
------------------
* indentation / code style
* added point cloud support (with ground filter) to humanoid_localization
* Added option to constrain motion according to odometry (don't estimate z / RP)
* cleanup, init localization from ground truth via tf lookup
* Fixed odometry lookup error (wrong frame) and missing odom data
* restructuring of humanoid_localization started
* Parameter cleanup / renaming
* Better height measurement in raycasting model
* humanoid_localization now uses sensor_msgs::Imu instead of nao_msgs
* EndpointModel is now optional, depends on dynamicEDT3D
* humanoid_localization added to humanoid_navigation, currently supports 6D localization from laser, odometry and IMU

* Contributors: Armin Hornung, Daniel Maier

