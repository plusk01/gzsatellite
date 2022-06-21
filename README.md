[![MIT License](https://img.shields.io/apm/l/atomic-design-ui.svg?)](https://github.com/plusk01/gzsatellite/blob/noetic-devel/LICENSE)
<img src="https://img.shields.io/badge/noetic-passing-green&style=plastic">

# Gazebo Satellite
![C++](https://img.shields.io/badge/-C%2B%2B-00599C?style=plastic&logo=C%2B%2B)
![CMake](https://img.shields.io/badge/-CMake-064F8C?style=plastic&logo=CMake)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

ROS/Gazebo plugin to create a satellite-imagery world based on a GPS location.


## Installation

* Install curl:

	  sudo apt install curl

* Clone the ROS package repository in your **src/** folder of your workspace.

      git clone https://github.com/plusk01/gzsatellite && cd ..

* Build your project.

      catkin build

* Finally run a test launch file. You can modify the coordinates of spawn here, for now the coordinates are set for the Rock Canyon Park.

      roslaunch gzsatellite test.launch

![Rock Canyon Park](https://user-images.githubusercontent.com/45683974/94589186-96080e80-02a2-11eb-9de5-8269363ad387.jpg)


## Considerations

This plugin allows you to pull in arbitrarily large satellite imagery into Gazebo.<br/>
Of course, that doesn't mean you should.<br/>
If you have a GPU, the large model that Gazebo creates could use a large portion of your VRAM for graphics rendering, causing other GPU processes (such as compute processes) to crash.<br/>
For example, pulling in a 400x400m region at zoom level 22 took ~1GB of VRAM for me. This did not leave enough resources for other GPU compute processes, causing crashes. If you have an NVIDIA GPU, you can check VRAM usage with the `nvidia-smi` command.<br/>
Combining this command with `watch -n 0.1 nvidia-smi` allows you to watch your GPU resources in real time.


###### 💾 EOF
