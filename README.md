# MARSIM
MARSIM: A light-weight point-realistic simulator for LiDAR-based UAVs

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

Ubuntu 22.04 with ROS 2 Humble is supported.

**Ten realistic maps (low and high resolution) have been realeased in the realease packages.**

## Prerequisited

### Ubuntu and ROS 2

Ubuntu 22.04 with ROS 2 Humble. [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html).

### PCL && Eigen && glfw3

PCL>=1.6, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

glfw3:
```
sudo apt-get install libglfw3-dev libglew-dev
```

### Make
```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone git@github.com:hku-mars/MARSIM.git
cd ..
colcon build
```

## Run single drone simulation

```
source install/setup.bash
ros2 launch test_interface single_drone_simple.launch.py
```
Click on 2D Goal Pose tool on the RViz2, you can give the UAV a position command to control its flight.

For now, we provide several launch files for users, which can be found in test_interface/launch folder.

You can change the parameter in launch files to change the map and LiDAR to be simulated. The maps have been uploaded to the realease files in this repository.

**Available maps can be found in the `map_generator/resource/` folder.**

**If you want to use the GPU version of MARSIM, please set the parameter "use_gpu_" to true.**

## Run single drone simulation with MID360 LiDAR
```
source install/setup.bash
ros2 launch test_interface single_drone_mid360.launch.py
```

## Acknowledgments
Thanks for [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL.git)

## Future
More realistic maps and functions are going to be released soon.