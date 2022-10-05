# metr4202teamproject
Team 13 for METR4202, S2 2022

## Build and Run
After cloning the repository, execute the following commands to build and run the project.
```bash
noetic  # source /opt/ros/noetic/setup.bash
# cd to the root directory of this repository
catkin build
source devel/setup.bash
```

At the moment, we run the project using
```bash
roslaunch project example.launch
```
but this will change as we progress into the project.

## Cloning
This project uses git submodules for dependency management.
Clone this repository with `git clone --recurse-submodules https://github.com/DukMastaaa/metr4202teamproject.git`.

The `serials` variable in `metr4202_ximea_ros/ximea_ros/src/ximea_demo.cpp` needs to be updated with our camera's serial number before building.