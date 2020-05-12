[![Build Status](https://travis-ci.org/RafaelRey/grid_map_tests.svg?branch=master)](https://travis-ci.org/RafaelRey/grid_map_tests)

# Grid Map Tests

This package is meant to start researching about the grid_map library from ETHZ and elevation mapping packages and discover how to integrate them in the NIx navigation stack.

Current State: Receiving a elevation map inside a grid map, trying to pass some filter to it and create a costmap from that. Filters are working but costmap it's created in a bad way, trying to solve it. 

Currently I'm using raposa bags such as:

https://www.dropbox.com/s/qd6hn0a0r1tlwwo/2020-01-03-20-32-03_undense.bag?dl=1

You can see what's happen by launching the included launch

## Dependencies

A lot :D. No, not much, you need to download [elevation mapping packages](https://github.com/anybotics/elevation_mapping), [kindr](https://github.com/anybotics/kindr), [kindr_ros](https://github.com/anybotics/kindr_ros), eigen, [grid_map library](https://github.com/anybotics/grid_map), costmap 2d from ros. Costmap 2d, grid map and eigen can be installed by apt:

```
sudo apt-get install ros-melodic-grid-map ros-melodic-costmap-2d libeigen3-dev
```

I'm currently using it under ros melodic, to use it under ros kinetic you should change the way the costmap wrapper object is initialize, beause it uses tf instead of tf2.
