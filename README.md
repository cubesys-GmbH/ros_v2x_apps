# Develop and run a sample application for cube-its

This repository provides examples demonstrating how to develop and run your own V2X (Vehicle-to-Everything) application on the *cube-its* within a [ROS 2 (Robot Operating System)](https://www.ros.org/) environment.

## Table of contents

1. [cube-its](#cube-its)
   - 1.1 [Component description](#component-description)
   - 1.2 [Compatible ETSI ITS messages](#compatible-ETSI-ITS-messages)
   - 1.3 [Conformance validation](#conformance-validation)
2. [ROS 2](#ros-2)
   - 2.1 [Node visibility](#node-visibility)
3. [Prerequisites](#prerequisites)
4. [Monitor Cooperative Awareness Messages](#monitor-cooperative-awareness-messages)
5. [Generate Decentralized Environmental Notification Messages](#generate-decentralized-environmental-notification-messages)

## cube-its <img src="https://img.shields.io/badge/ROS 2-jazzy | humble-blue"/>

The *cube-its* framework, as shown in figure 1, is designed for intelligent transportation systems (ITS) and vehicular networks within a ROS 2 environment.
It consists of several nodes and components that work together to manage GNSS data, vehicle kinematics, I/O operations, ITS facilities, and V2X communication using the [Vanetza](https://www.vanetza.org/) library.
Additionally, *cube-its* can serve as a platform for development, deployment and operation of ITS applications and beyond.

![Figure 1 - Schematic representation of cube-its](images/cube-its-schematic-architecture.png "Figure 1 - Schematic representation of cube-its")

### Component description

| Component | Description |
| --- | --- |
| GNSS | Provides accurate global positioning data for the system. It reads data from GNSS receiver and provides the position, velocity, and time information. | 
| Kinematics | Computes the kinematic state of the system based on GNSS data and other sensors. It calculates the system's pose, velocity, and acceleration. | 
| I/O | Handles sensor inputs and actuator outputs. It processes data from various sensors or interfaces such as CAN (Controller Area Network). | 
| ITS Facilities | Provides services and functionalities for intelligent transportation systems, including communication with traffic infrastructure and managing V2X communication. | 
| Vanetza | Facilitates V2X communication by implementing the **[ETSI (European Telecommunications Standards Institute)](https://www.etsi.org) ITS-G5** protocol for vehicle and infrastructure communication. | 

### Compatible ETSI ITS messages

The *cube-its* framework incorporates the [*etsi_its_messages*](https://github.com/ika-rwth-aachen/etsi_its_messages) package to facilitate the use of standardized ETSI ITS messages for V2X communication within ROS and ROS 2 environments. This integration enables developers to implement and manage V2X communication protocols, adhering to the ETSI specifications, within robotic and autonomous vehicle systems.

| Status | Acronym | Name | EN Specification | TS Specification |
| --- | --- | --- | --- | --- |
| :white_check_mark: | CAM | Cooperative Awareness Message | [EN 302 637-2 V1.4.1](https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.04.01_60/en_30263702v010401p.pdf) ([ASN.1](https://forge.etsi.org/rep/ITS/asn1/cam_en302637_2)) | - |
| :white_check_mark: | DENM | Decentralized Environmental Notification Message | [EN 302 637-3 V1.3.1](https://www.etsi.org/deliver/etsi_en/302600_302699/30263703/01.03.01_60/en_30263703v010301p.pdf) ([ASN.1](https://forge.etsi.org/rep/ITS/asn1/denm_en302637_3)) | - |
| :white_check_mark: | CPM | Collective Perception Message | - | [TS 103 324 V2.1.1](https://www.etsi.org/deliver/etsi_ts/103300_103399/103324/02.01.01_60/ts_103324v020101p.pdf) ([ASN.1](https://forge.etsi.org/rep/ITS/asn1/cpm_ts103324)) |
| :soon: | VAM | VRU Awareness Message | - | [TS 103 300-3 V2.2.1](https://www.etsi.org/deliver/etsi_ts/103300_103399/10330003/02.02.01_60/ts_10330003v020201p.pdf) |

### Conformance validation

The *cube-its* framework is validated using the ETSI conformance validation framework, as specified in [ETSI TR 103 099 V1.5.1](https://www.etsi.org/deliver/etsi_tr/103000_103099/103099/01.05.01_60/tr_103099v010501p.pdf).

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
  
# Monitor Cooperative Awareness Messages

![Figure 2 - Project cam_listener](images/cam_listener.png "Figure 2 - Project cam_listener")

The *cam_listener*, depicted in Figure 2, monitors for received Cooperative Awareness Messages (CAMs) sent through the designated published topic */its/cam_received* by *cube-its*. Within the *cube-its* framework, the publication of received CAM data is managed, while the *cam_listener* node is set up to subscribe to this particular topic. This setup allows the *cam_listener* node to receive and process CAM data, highlighting a key aspect of the project's functionality.

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
# Generate Decentralized Environmental Notification Messages

![Figure 3 - Project denm_node](images/denm_node.png "Figure 3 - Project denm_node")

The *denm_node*, illustrated in Figure 3, handles the transmission and reception of Decentralized Environmental Notification Messages (DENMs) via *cube-its*. It subscribes to specific topics to receive position updates and incoming DENMs, and it utilizes a service call to initiate the transmission of DENMs. Furthermore, the denm_node periodically generates and sends DENMs based on its current location.

## Subscriptions and Services
**Subscriptions:**
- **/its/position_vector:** The *denm_node* subscribes to this topic to receive regular updates about the current position.
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

# Generate Collective Perception Messages

![Figure 4 - Project cpm_provider](images/cpm_provider.png "Figure 4 - Project cpm_provider")

In the following example, we regularly create a Collective Perception Message (CPM) that includes sample Perceived Object data and transmits it based on the current position. The *cpm_provider*, illustrated in Figure 4, is tasked with supplying CPMs to *cube-its*. It subscribes to receive position updates and publishes a CPM to the */its/cpm_provided* topic, where the CPS facility in *cube-its* handles the transmission of the CPM. Furthermore, it consistently generates and sends CPMs according to the current position.

## Subscriptions and Publisher
**Subscriptions:**
- **/its/position_vector:** The *cpm_provider* subscribes to this topic to receive continuous updates regarding the current position.

**Publisher:**
- **/its/cpm_provided:** The *cpm_provider* provides the generated CPM to *cube-its* on this topic for transmission.

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

Now run the cpm_provider:

```
ros2 run v2x_apps cpm_provider
```
