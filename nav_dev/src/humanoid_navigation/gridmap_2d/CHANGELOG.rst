^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gridmap_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2018-04-19)
------------------
* refacotoring to release on kinetic
* added documents related to gridmap_2d packages
* added information new maintainer
* fixed CMakeLists.txt and package.xml files for dependency
* Contributors: Kayman, Pyo

0.4.1 (2016-09-05)
------------------
* Added include files for footstep_planner and gridmap_2d for install.
* gridmap_2d : OpenCV migration for indigo.
* gridmap_2d: Adding conversion back to OccupancyGrid msg
* gridmap_2d: adding external access to individual cells, allow recomputation of distance map
* catkinize the stack
* gridmap_2d: Add option to treat unknown as obstacle
* Adding copy c'tor to gridmap_2d, humanoid_planner_2d using local map copy now
* gridmap_2d: Fix map inflation

* Contributors: Armin Hornung, Pramuditha Aravinda, Vincent Rabaud, enriquefernandez

0.4.0 (2013-01-10)
------------------
* moved gridmap_2d into own namespace

* Contributors: Armin Hornung

0.3.1 (2012-06-15)
------------------
* Fixed gridmap_2d and footstep_planner to be used with ros fuerte and sbpl package

* Contributors: Armin Hornung

0.3.0 (2012-05-29)
------------------
* humanoid_navigation now compatible with electric and fuerte

* Contributors: Armin Hornung

0.2.0 (2011-11-28)
------------------
* added foundation for map change detection
* fixed invalid states appearing on border of map during expansion

* Contributors: Armin Hornung

0.1.0 (2011-05-03)
------------------
* Replace gridmap2D with other  cv::Mat
* Added a few more accessors to Gridmap2D
* Fixed OpenCV deprecation warnings
* Bugfix: x/y swapped in GridMap2D

* Contributors: Armin Hornung
