# Develop and run a sample application for cube-its

This repository provides examples demonstrating how to develop and run your own V2X (Vehicle-to-Everything) application on the *cube-its* within a [ROS 2 (Robot Operating System)](https://www.ros.org/) environment. The content is structured as follows: 

1. [cube-its](#cube-its)
   - 1.1 [Component description](docs/cube-its.md#component-description)
   - 1.2 [Compatible ETSI ITS messages](docs/cube-its.md#compatible-etsi-its-messages)
   - 1.3 [Conformance validation](docs/cube-its.md#conformance-validation)
   - 1.4 [Supported applications](docs/cube-its.md#supported-applications)
2. [ROS 2](#ros-2)
   - 2.1 [Node visibility](#node-visibility)
3. [Prerequisites](#prerequisites)
4. [Getting started](#getting-started)
5. [Code examples](docs/code_examples.md)
   - 5.1 [Cooperative Awareness Message (CAM)](docs/code_examples.md#cooperative-awareness-message)
   - 5.2 [Decentralized Environmental Notification Message (DENM)](docs/code_examples.md#decentralized-environmental-notification-message)
   - 5.3 [Collective Perception Message (CPM)](docs/code_examples.md#collective-perception-message)
   - 5.4 [Vulnerable Road User Awareness Message (VAM)](docs/code_examples.md#vulnerable-road-user-awareness-message)
   - 5.5 [Stationary Vehicle Warning (StVeWa)](docs/code_examples.md#stationary-vehicle-warning-trigger)
6. [Run tests](#run-tests)
7. [Real-world deployments](#real-world-deployments)

---

## cube-its <img src="https://img.shields.io/badge/latest-v1.4.0-green"/> <img src="https://img.shields.io/badge/ROS 2-jazzy | humble-blue"/> 

The **[cube-its](https://www.nfiniity.com/docs/dev/stacks/cube-its/intro/)** framework is designed to seamlessly integrate Intelligent Transportation Systems (ITS) applications and Vehicle-to-Everything (V2X) communication capabilities within a ROS 2 environment. It facilitates data exchange and communication between vehicles and external entities such as other vehicles, infrastructure, pedestrians, and cloud systems, leveraging V2X communication technologies.

The framework comprises multiple nodes and components that collaboratively handle GNSS data, vehicle kinematics, I/O operations, ITS facilities, and V2X communication, utilizing the [Vanetza](https://www.vanetza.org/) library, as illustrated in Figure 1.

Furthermore, *cube-its* serves as a comprehensive platform for the development, deployment, and operation of ITS applications and related innovations.

![Figure 1 - Schematic representation of cube-its](images/cube-its-schematic-architecture.png "Figure 1 - Schematic representation of cube-its")

For the framework details — components, interfaces, supported ETSI ITS messages, conformance and applications:

👉 [cube-its reference](docs/cube-its.md) · [official cube-its documentation](https://www.nfiniity.com/docs/dev/stacks/cube-its/intro/)

## ROS 2

ROS 2 is known as an advanced middleware for creating software for self-driving robots and even autonomous vehicles. It has a decentralized setup with nodes handling specific tasks, such as processing data from a single sensor etc. Since we will be working with ROS, it makes sense to get familiar with the environment and features of ROS.

### Node visibility 
In order to run ROS 2 nodes in the same ROS 2 environment, ROS 2 introduces a domain mechanism.
By default *ROS_LOCALHOST_ONLY* is set to 1, which means that *cube-its*, its topics, services, and actions will not be visible to other ROS 2 environments on the local network. 
By setting *ROS_LOCALHOST_ONLY=0* enables ROS 2 nodes from v2x_apps and *cube-its* to discover each other, if they share the same domain (default: *ROS_DOMAIN_ID=42*).
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

---

## Prerequisites
- A [cube device](https://www.nfiniity.com/#hardware-section) running the *cube-its* framework
- You likely already have worked with devcontainer projects in VSCode (Visual Studio Code). If you are not familiar with developing inside a container, check the following link https://code.visualstudio.com/docs/devcontainers/containers before you start.

---

## Getting started

These steps assume a running *cube-its* instance on a [cube:EVK or cube device](https://www.nfiniity.com/#hardware-section).

1. **Clone the repository and open it in the devcontainer.**

   ```bash
   git clone https://github.com/cubesys-GmbH/ros_v2x_apps.git
   ```

   Open the folder in VSCode and reopen it in the container when prompted.

2. **Build the package** from the workspace root.

   ```bash
   cd dev_ws
   colcon build --packages-select v2x_apps
   ```

3. **Source the overlay** so the executables are on your path.

   ```bash
   source install/setup.bash
   ```

4. **Enable discovery** so `v2x_apps` and *cube-its* see each other. They must share the same [`ROS_DOMAIN_ID`](#node-visibility) (default `42`).

   ```bash
   export ROS_LOCALHOST_ONLY=0
   ```

5. **Run a node** — `cam_listener` here; see [Code examples](docs/code_examples.md) for what each one does.

   ```bash
   ros2 run v2x_apps cam_listener
   ```

   Available nodes: `cam_listener`, `denm_node`, `cpm_provider`, `vam_provider`, `btp_listener`, `btp_sender`, `stationary_vehicle`.

The node started correctly when you see:

```
[INFO] [1706013094.349399714] [cam_listener]: Node "cam_listener" started
```

It then waits for CAMs from *cube-its*. Once they arrive:

```
[INFO] [1706013095.341824275] [cam_listener]: Received CAM from Station Id: 84281098
[INFO] [1706013096.345854233] [cam_listener]: Received CAM from Station Id: 84281098
[INFO] [1706013097.345731609] [cam_listener]: Received CAM from Station Id: 84281098
```

---

## Code examples

```bash
dev_ws
└── src/v2x_apps
    ├── package.xml
    ├── setup.cfg
    ├── setup.py
    ├── v2x_apps
    │   ├── btp_listener.py
    │   ├── btp_sender.py
    │   ├── cam_listener.py
    │   ├── cpm_provider.py
    │   ├── denm_node.py
    │   └── vam_provider.py
    ├── c2c
    │   └── stationary_vehicle_trigger.py
    └── test
        └── test_value_scaling.py
```

Each example node is documented separately, with a diagram and topic/service breakdown:

👉 [Code examples](docs/code_examples.md) — CAM, DENM, CPM, VAM and Stationary Vehicle Warning

---

## Run tests

From the root of the workspace, build and test:

```
colcon build --packages-select v2x_apps
colcon test --packages-select v2x_apps
colcon test-result --all --verbose
```

---

## Real-world deployments

Explore real-world scenarios that show how end users apply our technology in practical conditions.

👉 [Check out related use cases and demos](https://www.nfiniity.com/knowledge.html) 
