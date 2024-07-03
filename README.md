# Develop and run a sample application for cube-its

This project shows how to develop and run your own V2X (Vehicle-to-Everything) application on the *cube-its* in a [ROS 2 (Robot Operating System)](https://www.ros.org/) environment.

## cube-its

The *cube-its* is a collection of software tools, ROS 2 nodes and components and that provide functionalities for V2X (Vehicle-to-Everything) services, including facilities like CAMs (Cooperative Awareness Messages), DENMs (Decentralized Environmental Notification Messages), CPMs (Connection Protection Messages) and more. Additionally, *cube-its* can serve as a platform for development, deployment and operation of ITS applications and beyond.

## ROS 2

ROS 2 is known as an advanced middleware for creating software for self-driving robots and even autonomous vehicles. It has a decentralized setup with nodes handling specific tasks, such as processing data from a single sensor etc. Since we will be working with ROS, it makes sense to get familiar with the environment and features of ROS.

### Node visibility 
In order to run ROS 2 nodes in the same ROS 2 environment, ROS 2 introduces a domain mechanism.
By default *ROS_LOCALHOST_ONLY* is set to 1, which means that *cube-its*, its topics, services, and actions will not be visible to other ROS 2 environments on the local network. 
By setting *ROS_LOCALHOST_ONLY=0* enables ROS 2 nodes from v2x_apps and *cube_its* to discover each other, if they share the same domain (default: *ROS_DOMAIN_ID=42*).
You can simply disable the localhost only setting by typing in a terminal: 

```
export ROS_LOCALHOST_ONLY=0
```

In other words, they are part of the same ROS 2 environment, and the ROS 2 nodes from v2x_apps can now access to all services and topics of the *cube-its*.

In the same way, it's possible to set the domain to a different value:

```
export ROS_DOMAIN_ID=45
```

More information about domain ID can be found here: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html

## Prerequisites
- [cube:evk](https://www.nfiniity.com/#portfolio) or [cube:micro OBU](https://www.nfiniity.com/#portfolio) running the *cube-its* framework
- You likely already have worked with devcontainer projects in VSCode (Visual Studio Code). If you are not familiar with developing inside a container, check the following link https://code.visualstudio.com/docs/devcontainers/containers before you start.
  
# Project "cam_listener"

![Figure 1 - Project overview](images/cam_listener.png)

The *cam_listener*, as shown in figure 1, listens for received CAMs transmitted through the designated published topic "/its/cam_received" by *cube-its*. The *cube-its* framework handles the publication of received CAM data, while the *cam_listener* node is configured to subscribe to this specific topic. This configuration enables the *cam_listener* node to efficiently receive and process the CAM data, showcasing a fundamental aspect of the project's functionality.

The *cam_listener* node operates within a Docker container, similar to the *cube-its*. Both are functioning within a ROS 2 environment and share the same domain, facilitating the ability of ROS 2 nodes to discover each other.

## Build and run project

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


The node is running correctly when you see the following terminal output:

```
[INFO] [1706013094.349399714] [cam_listener]: Node "cam_listener" started
```

The *cam_listener* is now waiting for a received CAM message by *cube-its*. 
When *cube-its* starts receiving CAMs, *cam_listener* will output on terminal:

```
[INFO] [1706013095.341824275] [cam_listener]: Received CAM from Station Id: 84281098
[INFO] [1706013096.345854233] [cam_listener]: Received CAM from Station Id: 84281098
[INFO] [1706013097.345731609] [cam_listener]: Received CAM from Station Id: 84281098
[INFO] [1706013098.345113236] [cam_listener]: Received CAM from Station Id: 84281098
[INFO] [1706013099.344528362] [cam_listener]: Received CAM from Station Id: 84281098
```
# Project "denm_node"

![Figure 2 - Project overview](images/denm_node.png)

The *denm_node*, shown in figure 2, is responsible for transmitting and receiving DENMs over *cube-its*. The *denm_node* subscribes to topics to get position updates and received DENMs and uses a service call to request the transmission of DENMs. 
Additionally, it periodically generates and transmits DENMs based on the current position.

## Build and run project

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

Now run the denm_node:

```
ros2 run v2x_apps denm_node
```
