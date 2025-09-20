

# ACT-Edge

ACT-Edge is an engineering project specifically designed to deploy the embodied intelligence algorithm ACT onto edge devices (Horizon X5 RDK). 

The project is developed within the ROS 2 (C++) framework and implemented as a ROS 2 LifecycleNode, enabling dynamic loading and unloading, thereby greatly decoupling development and usage. 

In addition, To ensure efficiency and compatibility with the Horizon X5 RDK Infer API, all inference code is implemented in C++.

<!-- PROJECT SHIELDS -->

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

<!-- PROJECT LOGO -->
<br />

<p align="center">
  <a href="https://github.com/786739982/ACT-Edge/">
    <img src="assets/logo.png" alt="Logo" width="146" height="64">
  </a>

  <h3 align="center">ACT-Edge</h3>
  <p align="center">
    Deploy the Embodied AI onto Edge Devices ！
    <br />
    <a href="https://github.com/786739982/ACT-Edge"><strong>Explore the documentation of this project »</strong></a>
    <br />
    <br />
    <a href="https://github.com/786739982/ACT-Edge">Demo</a>
    ·
    <a href="https://github.com/786739982/ACT-Edge/issues">Report Bug</a>
    ·
    <a href="https://github.com/786739982/ACT-Edge/issues">Propose New Feature</a>
  </p>

</p>

<p align="center">
<img src="assets/ACT-Edge.gif", width=""/>
</p>

## 目录

- [Getting-Started](#Getting-Started)
  - [Requirements](#Requirements)
  - [Deployment](#Deployment)
- [Customization-Guide](#Customization-Guide)
  - Integrate Your Own Robotic Arm
  - Integrate Your Own Cameras
- [Author](#Author)
- [Acknowledgements](#Acknowledgements)




### Getting-Started

#### Requirements

1. [ROS2](https://docs.ros.org/en/rolling/index.html)
2. [Hobot DNN](https://developer.d-robotics.cc/rdk_doc/04_toolchain_development)

#### Deployment

Hardware

* USB Cameras
* AirbotPlay
* Horizon X5 RDK

Compile and Run
```
  # Compile
  cp ACT-Edge /your/ROS2_ws/src
  colcon build

  # Run
  # First setup your own Robotic Arm ROS2 Node，and then ...
  source /your/ROS2_ws/install/setup.bash
  ros2 run act_edge_pkg act_infer_app

  # Optional
  ros2 param set /act_infer_app namespace "your_namespace"
  
  # Activate lifecycle node
  ros2 lifecycle set /act_infer_app configure
  ros2 lifecycle set /act_infer_app activate

  # Deactivate lifecycle node
  ros2 lifecycle set /act_infer_app deactivate
  ros2 lifecycle set /act_infer_app cleanup

```

Get Quantized and Compiled Model
* One way: Refer to [Hobot DNN](https://developer.d-robotics.cc/rdk_doc/04_toolchain_development)
* Recommended way: Contact me for direct guidance, code and shell scripts. [My contact information](#Author). I also look forward to becoming friends with you!




### Customization-Guide

Since this project is implemented as a ROS 2 lifecycle node, you only need to implement your robotic arm and camera as ROS 2 nodes, and declare publishers and subscriptions to input raw data and receive inference results. 

So you can modify the corresponding publishers and subscriptions in this project to match your application.

#### Integrate Your Own Robotic Arm
```src/act_infer_app.cpp```
```
  joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        fmt::format("/{}/joint_states", namespace_), 10, std::bind(&Act_Infer_APP::joints_callback, this, std::placeholders::_1));
    
  joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(fmt::format("/{}/servo_node/joint_pos_cmds", namespace_), 10);
  
  gripper_publisher_ = this->create_publisher<airbot_controller_msgs::msg::GripperControl>(fmt::format("/{}/gripper_control", namespace_), 10);
```

#### Integrate Your Own Cameras
```src/act_infer_app.cpp```
```
  cam1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/usbcam1/color/image_bgr8", 10, callback_1);

  cam2_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/usbcam2/color/image_bgr8", 10, callback_2);
```




### Author

Hongrui Zhu 

E-Mail：786739982@qq.com or hongrui0226@gmail.com

qq:786739982

vx：Hong_Rui_0226



  
### 版权说明

该项目签署了MIT 授权许可，详情请参阅 [LICENSE](https://github.com/786739982/ACT-Edge/blob/master/LICENSE)





### Acknowledgements

- [DISCOVERSE](https://airbots.online/)




<!-- links -->
[contributors-shield]: https://img.shields.io/github/contributors/786739982/ACT-Edge.svg?style=flat-square
[contributors-url]: https://github.com/786739982/ACT-Edge/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/786739982/ACT-Edge.svg?style=flat-square
[forks-url]: https://github.com/786739982/ACT-Edge/network/members
[stars-shield]: https://img.shields.io/github/stars/786739982/ACT-Edge.svg?style=flat-square
[stars-url]: https://github.com/786739982/ACT-Edge/stargazers
[issues-shield]: https://img.shields.io/github/issues/786739982/ACT-Edge.svg?style=flat-square
[issues-url]: https://img.shields.io/github/issues/786739982/ACT-Edge.svg
[license-shield]: https://img.shields.io/github/license/786739982/ACT-Edge.svg?style=flat-square
[license-url]: https://github.com/786739982/ACT-Edge/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555





