# Humanoid Navigation

## ROS Packages for Humanoid Navigation
|Version|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|
|:---:|:---:|:---:|
|[![GitHub version](https://badge.fury.io/gh/ROBOTIS-GIT%2Fhumanoid_navigation.svg)](https://badge.fury.io/gh/ROBOTIS-GIT%2Fhumanoid_navigation)|[![Build Status](https://travis-ci.org/ROBOTIS-GIT/humanoid_navigation.svg?branch=kinetic-devel)](https://travis-ci.org/ROBOTIS-GIT/humanoid_navigation)|-|

## Wiki for humanoid_navigation Packages
- http://wiki.ros.org/humanoid_navigation (metapackage)
- http://wiki.ros.org/footstep_planner
- http://wiki.ros.org/gridmap_2d
- http://wiki.ros.org/humanoid_localization
- http://wiki.ros.org/humanoid_planner_2d

## Documents related to humanoid_navigation Packages
- [humanoid_navigation](http://wiki.ros.org/humanoid_navigation): ROS metapackages with footstep planning and localization for humanoid / biped robots. This metapackge contains subpackages like [footstep_planner](http://wiki.ros.org/footstep_planner), [gridmap_2d](http://wiki.ros.org/gridmap_2d), [humanoid_localization](http://wiki.ros.org/humanoid_localization), [humanoid_planner_2d](http://wiki.ros.org/humanoid_planner_2d).
  - Author: Armin Hornung, Johannes Garimort, Stefan Osswald, Daniel Maier
  - License: GPLv3, BSD
- [footstep_planner](http://wiki.ros.org/footstep_planner): This package a footstep planner for humanoid / biped robots. The planner builds on SBPL and has anytime as well as dynamic replanning capabilities. The supported planners are: ARA*, AD*, R*.
  - Author: Johannes Garimort, Armin Hornung
  - License: GPLv3
- [gridmap_2d](http://wiki.ros.org/gridmap_2d): This package is a simple 2D grid map structure, based on OpenCV's 'cv::Mat'.
  - Author: Armin Hornung
  - License: BSD
- [humanoid_localization](http://wiki.ros.org/humanoid_localization): 6D localization for humanoid robots based on depth data (laser, point clouds). Two observation models are currently available based on OctoMap as 3D map: Ray casting and an end point model (lookup in the distance map).
  - Author: Armin Hornung, Stefan Osswald, Daniel Maier
  - License: GPLv3
- [humanoid_planner_2d](http://wiki.ros.org/humanoid_planner_2d): Thi package provides a simple 2D path planner as wrapper around SBPL (ARA*, AD*, R*).
  - Author: Armin Hornung
  - License: BSD
- Papers related to these packages:
  ```
  "Humanoid robot localization in complex indoor environments",
  by A. Hornung, K. M. Wurm and M. Bennewitz,
  2010 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2010, pp. 1690-1695.
  doi: 10.1109/IROS.2010.5649751
  ```
  ```
  "Humanoid navigation with dynamic footstep plans",
  by J. Garimort, A. Hornung and M. Bennewitz,
  2011 IEEE International Conference on Robotics and Automation (ICRA), 2011, pp. 3982-3987.
  doi: 10.1109/ICRA.2011.5979656
  ```
  ```
  "Anytime search-based footstep planning with suboptimality bounds", 
  by A. Hornung, A. Dornbush, M. Likhachev and M. Bennewitz,
  2012 12th IEEE-RAS International Conference on Humanoid Robots (Humanoids 2012), 2012, pp. 674-679.
  doi: 10.1109/HUMANOIDS.2012.6651592
  ```

---

## Notice for Original Source Code and author, maintainer

These packages are a modified version by forking the following [humanoid_navigation](https://github.com/ahornung/humanoid_navigation) package by ROBOTIS.
Please refer to the following links for original information.

- Original Source Code by Armin Hornung (Electric ~ Hydro Version)
  - Repository: https://github.com/ahornung/humanoid_navigation 
- Source code for maintenance on ROS Indigo Version (by Pramuditha Aravinda)
  - Repository: https://github.com/AravindaDP/humanoid_navigation
  - Issue related: https://github.com/ahornung/humanoid_navigation/issues/14
- Source code for maintenance on ROS Kinetic Version (by Pyo)
  - Repository: https://github.com/ROBOTIS-GIT/humanoid_navigation
  - Issue related: https://github.com/AravindaDP/humanoid_navigation/issues/5
