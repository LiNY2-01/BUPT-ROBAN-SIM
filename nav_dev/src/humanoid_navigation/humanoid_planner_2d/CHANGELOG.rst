^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package humanoid_planner_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2018-04-19)
------------------
* refacotoring to release on kinetic
* added documents related to humanoid_planner_2d packages
* added information new maintainer
* fixed CMakeLists.txt and package.xml files for dependency
* Contributors: Kayman, Pyo

0.4.1 (2016-09-05)
------------------
* catkinize the stack
* humanoid_planner_2d on gridmaps, provides a lightweight wrapper around SBPL

* enhancements:

  * check if goal / start reachable before planning
  * adding option to find SBPL overlay, reducing output
  * Getters for robot (inflation) radius, path costs from SBPL
  * Add another planning method (double)

* fixes:

  * Fix map inflation for planning

* Contributors: Armin Hornung, Vincent Rabaud, enriquefernandez

