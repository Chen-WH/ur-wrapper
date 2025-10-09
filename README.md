# ur_wrappers

## 1 Setup PC

1. Connect the UR control box directly to the remote PC with an ethernet cable. On the remote PC, turn off all network devices except the “wired connection”, e.g. turn off wifi. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:

```yaml
IPv4
Manual
Address: 192.168.125.126
Netmask: 255.255.255.0
Gateway: 192.168.125.1
DNS: 192.168.125.1
```

2. Verify the connection from the PC with e.g. ping.

```shell
ping 192.168.125.6
```

## 2* ROS2 Driver

推荐优先使用Sec 3中的C++ SDK

[UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
[UR ROS2 Driver Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html)

1. Install the driver

```shell
sudo apt-get install ros-${ROS_DISTRO}-ur
sudo apt install ros-${ROS_DISTRO}-ros2-control \
                 ros-${ROS_DISTRO}-ros2-controllers \
                 ros-${ROS_DISTRO}-gazebo-ros2-control
```

2. 开启上位机ros2节点，UR可以给上位机发话题（可以读到关节角）

```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.125.6
```

3. 在UR示教器上加载程序（上位机可以控制UR）并运行

```shell
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{
      positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 
      time_from_start: {sec: 5}
    }]
  }
}"
```

4. 读取关节角度

```shell
ros2 topic echo /joint_states
```

TIP: joint_states话题下顺序是2-6,最后是1，注意！

## 3 Client Library

[UR Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library)

[UR Client Library Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/index.html#ur-client-library)

```shell
sudo apt install ros-$ROS_DISTRO-ur-client-library
```

1. 位置伺服

MODE_SERVOJ，可以参考full_driver.cpp

2. 速度/加速度前馈

MOVEJ，可以参考trajectory_point_interface.cpp