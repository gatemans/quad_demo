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






