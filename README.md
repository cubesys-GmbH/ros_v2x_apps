# Introduction

This project shows how to develop and run your own V2X (Vehicle-to-Everything) application on the *cube-its* in a [ROS 2 (Robot Operating System)](https://www.ros.org/) environment.

ROS 2 is known as an advanced middleware for creating software for self-driving robots and even autonomous vehicles. It has a decentralized setup with nodes handling specific tasks, such as processing data from a single sensor etc. Since we will be working with ROS, it makes sense to get familiar with the environment and features of ROS.

The *cube-its* is a collection of software tools, ROS 2 nodes and components and that provide functionalities for V2X (Vehicle-to-Everything) services, including facilities like CAMs (Cooperative Awareness Messages), DENMs (Decentralized Environmental Notification Messages), CPMs (Connection Protection Messages) and more. Additionally, *cube-its* can serve as a platform for development, deployment and operation of ITS applications and beyond.

## Project overview

![Figure 1 - Project overview](images/cam_listener.png)

The sample application features a simple ROS 2 node named *cam_listener* as shown figure 1. This node listens for received CAMs transmitted through the designated published topic "/vanetza/cam_received" by *cube-its*. The *cube-its* framework handles the publication of received CAM data, while the *cam_listener* node is configured to subscribe to this specific topic. This configuration enables the *cam_listener* node to efficiently receive and process the CAM data, showcasing a fundamental aspect of our project's functionality.

The *cam_listener* node operates within a Docker container, similar to the *cube-its*. Both are functioning within a ROS 2 environment and share the same domain, facilitating the ability of ROS 2 nodes to discover each other.

## Build project

You likely already have open the devcontainer project with Visual Studio Code. 
Navigate to the root of the workspace, dev_ws:

```
cd dev_ws
```

Build application by:

```
colcon build --packages-select v2x_apps
```

Still in the same terminal, source the setup files:

```
source install/setup.bash
```

Now run the cam_listener node:

```
ros2 run v2x_apps cam_listener
```