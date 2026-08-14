## quad_demo
# simple demo of quarrotor in ros lecture
this is a simple ros project for exaplaining of "node" , "topic" , "Action" and "service" 


```

                         ┌─────────────────────────────┐
                         │  go_to_point.py             │
                         │  Action Client              │
                         └──────────────┬──────────────┘
                                        │ Action Goal
                                        ▼
                         ┌─────────────────────────────┐
                         │  controller.py              │
                         │  Action Server + Controller │
                         └──────────────┬──────────────┘
                                        │ /drone/cmd_vel
                                        │ Topic
                                        ▼
┌───────────────────┐   ┌─────────────────────────────┐
│ telemetry_monitor │◄──│  drone_sim.py               │
└───────────────────┘   │                             │
       ▲                └──────────┬──────────┬───────┘
       │                           │          │
       │ /drone/pose               │          │ Services
       │ /drone/velocity           │          ├─ /drone/arm
       │ Topics                    │          └─ /drone/set_pose
       └───────────────────────────┘

```

# first soure the env
```
source /opt/ros/humble/setup.bash
```
To make this setting permanent:
```
echo "source /opt/ros/humble/setup.bash" 
```

and check version
```
printenv ROS_DISTRO
```

# Make the wrk space
```
mkdir -p ~/quad_ws/src
cd ~/quad_ws
```
```
cd ~/quad_ws/src
ros2 pkg create --build-type ament_cmake quad_interfaces --dependencies geometry_msgs

```
the structure

```
quad_control/
├── package.xml
├── resource/
├── setup.cfg
├── setup.py
└── quad_control/
    └── __init__.py

```
# quad_interfaces Package:
```
mkdir -p ~/quad_ws/src/quad_interfaces
cd ~/quad_ws/src/quad_interfaces
mkdir srv action

```

# Creating an Action named GoTo
```
geometry_msgs/Point target
float32 cruise_speed

---
==
bool success
string message

---

geometry_msgs/Point current_position
float32 current_speed
float32 distance_remaining

```
# Create a Service named SetPose:
```
geometry_msgs/Point pose
---
bool success
string message

```
# Edit the CMakeLists.txt and  package.xml of the Interface package:


- - - - - 
# Add three Python files to it:
```
cd ~/quad_ws/src/quad_control/quad_control
touch quad_controller.py
touch state_monitor.py
touch mission_manager.py
```



