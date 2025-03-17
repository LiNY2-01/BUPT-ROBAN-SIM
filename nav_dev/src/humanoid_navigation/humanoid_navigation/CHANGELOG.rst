^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package humanoid_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* humanoid_navigation : removed hrl_kinematics from runtime dependancy
* catkinize the stack
* humanoid_planner_2d on gridmaps, provides a lightweight wrapper around SBPL

* Contributors: Armin Hornung, Vincent Rabaud, enriquefernandez

0.4.0 (2013-01-10)
------------------
* humanoid_localization added to humanoid_navigation, currently supports 6D localization from laser, odometry and IMU

* Contributors: Armin Hornung, Daniel Maier, Johannes Garimort

0.3.1 (2012-06-15)
------------------
* Fixed gridmap_2d and footstep_planner to be used with ros fuerte and sbpl package

* Contributors: Armin Hornung, Johannes Garimort

0.3.0 (2012-05-29)
------------------
* humanoid_navigation now compatible with electric and fuerte

* Contributors: Armin Hornung, Johannes Garimort

0.2.0 (2011-11-28)
------------------
* stack.xml and manifest updated in footstep_planner to use SBPL instead of D* lite, enabling ARA* and AD* for search with anytime capabilities

* Contributors: Armin Hornung, Johannes Garimort

0.1.0 (2011-05-03)
------------------
* New humanoid_navigation stack created

* Contributors: Armin Hornung, Johannes Garimort
