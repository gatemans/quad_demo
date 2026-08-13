## quad_demo
# simple demo of quarrotor in ros lecture


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
