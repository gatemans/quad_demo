## quad_demo
# simple demo of quarrotor in ros lecture
this is a simple ros project for exaplaining of "node" , "topic" , "Action" and "service" 




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
‍‍```

```

```
ros2 pkg create quad_interfaces --build-type ament_cmake
ros2 pkg create quad_control --build-type ament_python --dependencies rclpy geometry_msgs std_srvs

```
the structure

```
quad_ws/
├── src/
│   ├── quad_interfaces/
│   └── quad_controller/
├── build/
├── install/
└── log/
```
# quad_interfaces Package:
```
mkdir -p ~/quad_ws/src/quad_interfaces
cd ~/quad_ws/src/quad_interfaces
mkdir srv action

```

source file:
```
nano srv/SetStart.srv
```
action file:
```
nano action/GoToPoint.action
```







