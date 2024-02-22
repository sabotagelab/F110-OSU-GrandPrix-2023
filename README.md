# f1ten-competition codebase

This codebase contains following ROS packages developed for autonomous racing competition in 2023.

1. System:

    a. Serial:
        A serial library to handle communication with hardware.
        Link for detailed documentation: https://github.com/wjwwood/serial

    b. URG Node:
        ROS wrapper for the Hokuyo urg_c library.
        Link to repository: https://github.com/ros-drivers/urg_node

    c. Racecar:
        Brings up the car by launching all the necessary nodes.
        Contains multiplexer for input commands which is used to decide priorities of the inputs coming from different sources.
        Forked from: https://github.com/mit-racecar/racecar

2. Localization and Mapping:
    a. Particle filter:
        Python implementation of particle filter modified and tuned for the F110 platform.
        The implementation uses RangeLibc for accelerated ray casting.
        Orinal implementation of range_libc can be found at: 
        https://github.com/kctess5/range_libc
    
    b. LaserScanMatcher and SLAM Gmapping are used but not included in the repository

3. PP variants:
    
    This package contains Pure Pursuit (PP) implementation, Autonomous emergency breaking and dynamic obstacle avoidance.




### Dependencies

Required:
* [catkin](http://www.ros.org/wiki/catkin) - cmake and Python based buildsystem
* [cmake](http://www.cmake.org) - buildsystem
* [Python](http://www.python.org) - scripting language
  * [empy](http://www.alcyone.com/pyos/empy/) - Python templating library
  * [catkin_pkg](http://pypi.python.org/pypi/catkin_pkg/) - Runtime Python library for catkin
* [gmapping](https://github.com/ros-perception/slam_gmapping/tree/melodic-devel) - SLAM Gmapping