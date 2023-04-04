# MARSIM
MARSIM: A light-weight point-realistic simulator for LiDAR-based UAVs
## Contents
[Achievements/ and/ Introduction](#Briefly Introduction and Achievements)
## Briefly Introduction and Achievements
1. Reconstruct cloud maps from real environments scanned by LiDARs
2. Lower Hardware platform required; the simulation could be processed on PC with GPU acceleration
3. Supporting three types of dynamic obstacles, multi-UAV system simulations, and mechanical spinning LiDAR Models

Paper is available on Arxiv: https://arxiv.org/abs/2211.10716

The video is available on youtube: https://youtu.be/hiRtcq-5lN0 and 
【MARSIM: 轻量化雷达无人机仿真器】 https://www.bilibili.com/video/BV1M84y117KG

<p align="center">
  <a href="https://youtu.be/hiRtcq-5lN0" target="_blank"><img src="figures/coverfigure.png" alt="video" width="800" height="400" border="1" /></a>
</p>

<p align="center">

  <img src="figures/readme_setgoal.gif" width = "400" height = "237"/>

  <img src="figures/readme_dynobs.gif" width = "400" height = "237"/>


  <img src="figures/readme_multiuav.gif" width = "400" height = "237"/>


  <img src="figures/readme_exploration.gif" width = "400" height = "237"/>
</p>

## Update

Ubuntu 20.04 is also supported in ubuntu20 branch.

**Ten realistic maps (low and high resolution) have been realeased in the realease packages.**

**A new branch that merge with FUEL has been released in the fuel_ubuntu20 branch.**



## Prerequisited

### Ubuntu and ROS

Ubuntu 16.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### PCL && Eigen && glfw3

PCL>=1.6, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

glfw3:
```
sudo apt-get install libglfw3-dev libglew-dev
```

### Make
```
mkdir -p marsim_ws/src
cd marsim_ws/src
git clone git@github.com:hku-mars/MARSIM.git
cd ..
catkin_make
```

## Run single drone simulation

```
source devel/setup.bash
roslaunch test_interface single_drone_avia.launch
```
Click on 3Dgoal tool on the Rviz, you can give the UAV a position command to control its flight.

For now, we provide several launch files for users, which can be found in test_interface/launch folder.

You can change the parameter in launch files to change the map and LiDAR to be simulated. The maps have been uploaded to the realease files in this repository.

```
    <arg name="map_name" value="$(find map_generator)/resource/small_forest01cutoff.pcd"/>

```

**If you want to use the GPU version of MARSIM, please set the parameter "use_gpu" to true.**

## Run single drone simulation with dynamic obstacles
```
source devel/setup.bash
roslaunch test_interface single_drone_mid360_dynobs.launch
```

## Run multiple drones simulation
```
source devel/setup.bash
roslaunch test_interface triple_drone_mid360.launch
```

## Run the simulation with FUEL algorithm

You should first change the branch to fuel_ubuntu20 branch. If you are using ubuntu 20.04, you should first download Nlopt and make install it in your environment. Then you can run the simulation by the command below:
```
source devel/setup.bash
roslaunch exploration_manager exploration.launch
```
Then click on 2Dgoal tool on the Rviz, randomly click on the map, and FUEL would automously run.

## Acknowledgments
Thanks for [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL.git)

## Future
More realistic maps and functions are going to be released soon.
