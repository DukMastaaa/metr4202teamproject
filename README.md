# metr4202teamproject
Team 13 for METR4202, S2 2022

## Build and Run
After cloning the repository, execute the following commands to build and run the project.
```bash
noetic  # source /opt/ros/noetic/setup.bash
# cd to the root directory of this repository (metr4202teamproject)
catkin build
source devel/setup.bash
```
The launcher is accessed through "robot_launch full.launch". The camera and servo motor must be set-up through shell commands.
There is a prompt in robot_planner that will ask to run the fun task - enter y to run this or n otherwise.

## Cloning
This project uses git submodules for dependency management.
Clone this repository with `git clone --recurse-submodules https://github.com/DukMastaaa/metr4202teamproject.git`.

The `serials` variable in `metr4202_ximea_ros/ximea_ros/src/ximea_demo.cpp` needs to be updated with our camera's serial number before building.
