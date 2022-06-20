# Chance Constrained Model Predictive Control

<img src="figures/drone.jpg" width="800">

Chance constrained MPC for unmanned aerial vehicle flight control with consideration of wind speed and direction.

This MPC is intended to be used with a dispersion model identified through flight experiments. The model predicts the distribution of vermiculite dispensed from a hovering drone given the drone’s movement state, wind condition, and dispenser setting.

A video of the experiment can be found here: [**UAV-based Precision Pest Management: Vermiculite Dispensing Experiment with Bugbot**](https://www.youtube.com/watch?v=st_apuEBtJg)

## Publication
Na Ma, Anil Mantri, Graham Bough, Ayush Patnaik, Siddhesh Yadav, Christian Nansen and Zhaodan Kong, Data-Driven Vermiculite Distribution Modelling for UAV-Based Precision Pest Management, Frontiers in Robotics and AI (peer-reviewing).

## Requirement
This code has only been tested on Ubuntu 20.04 with ROS Noetic. ROS2 currently is not supported.

## Installation
#### Please follow these steps to install **ROS Noetic**:
**Step 1**. Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

**Step 2**. Set up your keys.
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

**Step 3**. First, make sure your Debian package index is up-to-date.
```
sudo apt update
```
Now install the ROS:
```
sudo apt install ros-noetic-desktop-full
```

**Step 4**. You must source this script in every bash terminal you use ROS in. 
```
source /opt/ros/noetic/setup.bash
```

**Step 5**. To install this tool and other dependencies for building ROS packages, run:
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

**Step 6**. Install `rosdep` and initialize `rosdep`.
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```