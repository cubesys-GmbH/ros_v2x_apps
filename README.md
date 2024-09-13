# Develop and run a sample application for cube-its

This repository provides examples demonstrating how to develop and run your own V2X (Vehicle-to-Everything) application on the *cube-its* within a [ROS 2 (Robot Operating System)](https://www.ros.org/) environment.

## cube-its <img src="https://img.shields.io/badge/ROS 2-humble-blue"/>

The *cube-its* framework, as shown in figure 1, is designed for intelligent transportation systems (ITS) and vehicular networks within a ROS 2 environment.
It consists of several nodes and components that work together to manage GNSS data, vehicle kinematics, I/O operations, ITS facilities, and V2X communication using the [Vanetza](https://www.vanetza.org/) library.
Additionally, *cube-its* can serve as a platform for development, deployment and operation of ITS applications and beyond.

![Figure 1 - Schematic representation of cube-its](images/cube-its-schematic-architecture.png "Figure 1 - Schematic representation of cube-its")

### Component description 

#### GNSS
The *GNSS* component provides accurate global positioning data for the system. It reads data from GNSS receiver and provides the position, velocity, and time information.

#### Kinematics
The *Kinematics* computes the kinematic state of the system based on GNSS data and other sensors. It calculates the system's pose, velocity, and acceleration.

#### I/O
The *I/O* components handle sensor inputs and actuator outputs. It processes data from various sensors or interfaces such as CAN (Controller Area Network).

#### ITS Facilities
The *ITS Facilities* provides services and functionalities for intelligent transportation systems, including communication with traffic infrastructure and managing V2X communication.

#### Vanetza
The *Vanetza* node facilitates V2X communication by implementing the **[ETSI (European Telecommunications Standards Institute)](https://www.etsi.org) ITS-G5** protocol for vehicle and infrastructure communication.

### Compatible ETSI ITS messages and compliance

The *cube-its* framework incorporates the [*etsi_its_messages*](https://github.com/ika-rwth-aachen/etsi_its_messages) package to facilitate the use of standardized ETSI ITS messages for V2X communication within ROS and ROS 2 environments. This integration enables developers to implement and manage V2X communication protocols, adhering to the ETSI specifications, within robotic and autonomous vehicle systems.

| Status | Acronym | Name | EN Specification | TS Specification | TR Specification |
| --- | --- | --- | --- | --- |--- |
| :white_check_mark: | CAM | Cooperative Awareness Message | [EN 302 637-2 V1.4.1](https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.04.01_60/en_30263702v010401p.pdf) ([ASN.1](https://forge.etsi.org/rep/ITS/asn1/cam_en302637_2)) | - | [ETSI TR 103 099 V1.5.1](https://www.etsi.org/deliver/etsi_tr/103000_103099/103099/01.05.01_60/tr_103099v010501p.pdf) | 
| :white_check_mark: | DENM | Decentralized Environmental Notification Message | [EN 302 637-3 V1.3.1](https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.03.01_60/en_30263703v010301p.pdf) ([ASN.1](https://forge.etsi.org/rep/ITS/asn1/denm_en302637_3)) | - |[ETSI TR 103 099 V1.5.1](https://www.etsi.org/deliver/etsi_tr/103000_103099/103099/01.05.01_60/tr_103099v010501p.pdf) |
| :soon: | CPM | Collective Perception Message | - | [TS 103 324 V2.1.1](https://www.etsi.org/deliver/etsi_ts/103300_103399/103324/02.01.01_60/ts_103324v020101p.pdf) ([ASN.1](https://forge.etsi.org/rep/ITS/asn1/cpm_ts103324)) | - |

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

![Figure 2 - Project cam_listener](images/cam_listener.png "Figure 2 - Project cam_listener")

The *cam_listener*, as shown in figure 2, listens for received CAMs transmitted through the designated published topic "/its/cam_received" by *cube-its*. The *cube-its* framework handles the publication of received CAM data, while the *cam_listener* node is configured to subscribe to this specific topic. This configuration enables the *cam_listener* node to efficiently receive and process the CAM data, showcasing a fundamental aspect of the project's functionality.

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

![Figure 3 - Project denm_node](images/denm_node.png "Figure 3 - Project denm_node")

The *denm_node*, shown in figure 3, is responsible for transmitting and receiving DENMs over *cube-its*. The *denm_node* subscribes to topics to get position updates and received DENMs and uses a service call to request the transmission of DENMs. 
Additionally, it periodically generates and transmits DENMs based on the current position.

## Subscriptions and Services
**Subscriptions:**
- **/its/position_vector:** The denm_node subscribes to this topic to receive regular updates about the current position.
- **/its/denm_received:** This subscription allows the *denm_node* to receive incoming DENMs from other V2X capable stations. By processing these messages, the node can react to various environmental events and updates.

**Services:**
- **/its/den_request:** The *denm_node* can use this service to request the transmission of a DENM. This is likely an on-demand feature, where a specific condition or event triggers the need to send a DENM immediately. Here, in this example the transmission is called periodically.

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
