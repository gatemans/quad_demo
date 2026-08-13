## quad_demo
# simple demo of quarrotor in ros lecture
this is a simple ros project for exaplaining of "node" , "topic" , "Action" and "service" 



```
                 Service: /set_start
       ┌─────────────────────────────┐
       │                             ▼
  ┌─────────────┐                ┌──────────────────┐
  │ User / CLI  │                │                  │
  └─────────────┘                │  controller_node │
         │ Action: /go_to_point  │                  │
         └──────────────────────►│                  │
                                 └───────┬─────┬────┘
                                         │     │
                         Topic: /quad/pose     │ Topic: /quad/velocity
                                         │     │
                                         ▼     ▼
                                  ┌─────────────────┐
                                  │  monitor_node   │
                                  └─────────────────┘
```

# first soure the env
```
source /opt/ros/humble/setup.bash
```
To make this setting permanent:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

and check version
```
printenv ROS_DISTRO
```

# Make the wrk space
```
mkdir -p ~/quad_ws/src
cd ~/quad_ws/src
```

```
quad_ws/
├── src/
│   ├── quad_interfaces/
│   └── quad_controller/
├── build/
├── install/
└── log/
```



# quad_demo

A simple ROS 2 project for explaining:

- Node
- Topic
- Service
- Action

This project simulates a quadrotor moving from a start point to a target point.

## Architecture
text
Service: /set_start
┌─────────────────────────────┐
│                             ▼
  ┌─────────────┐                ┌──────────────────┐
  │ User / CLI  │                │                  │
  └─────────────┘                │  controller_node │
│ Action: /go_to_point  │                  │
└──────────────────────►│                  │
└───────┬─────┬────┘
│     │
Topic: /quad/pose     │ Topic: /quad/velocity
│     │
▼     ▼
┌─────────────────┐
│  monitor_node   │
└─────────────────┘
Requirements
Ubuntu 22.04
ROS 2 Humble
Python 3
colcon
rosdep
Source ROS 2

content_copy
bash

note_add
ویرایش با Canvas
source /opt/ros/humble/setup.bash
Make this setting permanent:


content_copy
bash

note_add
ویرایش با Canvas
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
Check the ROS 2 version:


content_copy
bash

note_add
ویرایش با Canvas
printenv ROS_DISTRO
Expected output:


content_copy
text

note_add
ویرایش با Canvas
humble
Create the Workspace

content_copy
bash

note_add
ویرایش با Canvas
mkdir -p ~/quad_ws/src
cd ~/quad_ws/src
Workspace structure:


content_copy
text

note_add
ویرایش با Canvas
quad_ws/
├── src/
│   ├── quad_interfaces/
│   └── quad_controller/
├── build/
├── install/
└── log/
Package Structure

content_copy
text

note_add
ویرایش با Canvas
quad_interfaces/
├── action/
│   └── GoToPoint.action
├── srv/
│   └── SetStart.srv
├── CMakeLists.txt
└── package.xml

content_copy
text

note_add
ویرایش با Canvas
quad_controller/
├── quad_controller/
│   ├── __init__.py
│   ├── controller_node.py
│   ├── monitor_node.py
│   └── goal_client.py
├── resource/
│   └── quad_controller
├── package.xml
├── setup.cfg
└── setup.py
Create the Interface Package

content_copy
bash

note_add
ویرایش با Canvas
cd ~/quad_ws/src

ros2 pkg create quad_interfaces --build-type ament_cmake

cd quad_interfaces

mkdir srv action
Service Definition
Create the service file:


content_copy
bash

note_add
ویرایش با Canvas
nano srv/SetStart.srv
Add:


content_copy
srv
float64 x
float64 y
float64 z
---
bool success
string message
Action Definition
Create the action file:


content_copy
bash

note_add
ویرایش با Canvas
nano action/GoToPoint.action
Add:


content_copy
action
float64 x
float64 y
float64 z
---
bool success
string message
---
geometry_msgs/Point current_position
float64 distance_remaining
Create the Controller Package

content_copy
bash

note_add
ویرایش با Canvas
cd ~/quad_ws/src

ros2 pkg create quad_controller \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs
Add the following dependency to quad_controller/package.xml:


content_copy
xml

note_add
ویرایش با Canvas
<depend>quad_interfaces</depend>
Build the Workspace

content_copy
bash

note_add
ویرایش با Canvas
cd ~/quad_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build
Source the workspace:


content_copy
bash

note_add
ویرایش با Canvas
source install/setup.bash
For a clean build:


content_copy
bash

note_add
ویرایش با Canvas
cd ~/quad_ws

rm -rf build install log

colcon build

source install/setup.bash
Check the Interfaces
Check the service:


content_copy
bash

note_add
ویرایش با Canvas
ros2 interface show quad_interfaces/srv/SetStart
Check the action:


content_copy
bash

note_add
ویرایش با Canvas
ros2 interface show quad_interfaces/action/GoToPoint
Run the Project
Open four terminals.

Source the environment in every terminal:


content_copy
bash

note_add
ویرایش با Canvas
source /opt/ros/humble/setup.bash
source ~/quad_ws/install/setup.bash
Terminal 1: Controller Node

content_copy
bash

note_add
ویرایش با Canvas
ros2 run quad_controller controller_node
Terminal 2: Monitor Node

content_copy
bash

note_add
ویرایش با Canvas
ros2 run quad_controller monitor_node
Terminal 3: Set the Start Point

content_copy
bash

note_add
ویرایش با Canvas
ros2 service call /set_start \
quad_interfaces/srv/SetStart \
"{x: 0.0, y: 0.0, z: 1.0}"
Terminal 4: Send a Goal

content_copy
bash

note_add
ویرایش با Canvas
ros2 run quad_controller goal_client 4 3 2
Send a Goal Using the ROS 2 CLI

content_copy
bash

note_add
ویرایش با Canvas
ros2 action send_goal --feedback \
/go_to_point \
quad_interfaces/action/GoToPoint \
"{x: 2.0, y: 5.0, z: 1.0}"
Useful ROS 2 Commands
List active nodes:


content_copy
bash

note_add
ویرایش با Canvas
ros2 node list
List topics:


content_copy
bash

note_add
ویرایش با Canvas
ros2 topic list
Show topic type:


content_copy
bash

note_add
ویرایش با Canvas
ros2 topic type /quad/pose
Read the position topic:


content_copy
bash

note_add
ویرایش با Canvas
ros2 topic echo /quad/pose
Read the velocity topic:


content_copy
bash

note_add
ویرایش با Canvas
ros2 topic echo /quad/velocity
List services:


content_copy
bash

note_add
ویرایش با Canvas
ros2 service list
List actions:


content_copy
bash

note_add
ویرایش با Canvas
ros2 action list
List installed packages:


content_copy
bash

note_add
ویرایش با Canvas
ros2 pkg list | grep quad
ROS 2 Concepts
Node
An independent ROS 2 program.

Topic
A continuous data channel.

Service
A request and response communication method.

Action
A communication method for long-running tasks.

An action has:

Goal
Feedback
Result
Topics

content_copy
Topic	Message Type	Description
/quad/pose	geometry_msgs/PoseStamped	Current quadrotor position
/quad/velocity	geometry_msgs/Twist	Current quadrotor velocity
Service

content_copy
Service	Type	Description
/set_start	quad_interfaces/srv/SetStart	Set the start position
Action

content_copy
Action	Type	Description
/go_to_point



