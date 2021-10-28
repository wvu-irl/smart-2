# SMART 2

[![MIT License][license-shield]][license-url]


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!--About The Project -->
## About the project
Repository for SMART 2.0 robot, a platform build on top of iRobot create robot, running ROS in a x64 Intel NUC computer, ADIS IMU and other sensors


<p align="right">(<a href="#top">back to top</a>)</p>

## Getting Started

The code was tested in Ubuntu 18.04 and Ubuntu 20.04 and ROS Melodic and Noetic

### Prerequisites

For instructions installing ROS:
http://wiki.ros.org/noetic/Installation/Ubuntu

For running the MAE493 course code matlab is necessary.

Install usb camera and create dependencies:
```sh
 sudo apt-get install ros-melodic-libcreate ros-melodic-usb-cam
```

Installing ssh server is usefull for accessing the robot computer for running and monitoring the code.
```sh
sudo apt-get install openssh-server
```

### Installation

1. Add user to the dialout group for serial communication with the different sensors and resart the computer
y
```sh
 $ usermod -a -G dialout MY_USER_NAME
```


2. Create a ROS workspace and go to the src directory
```sh
  mkdir my_smart_ws
  mkdir my_smart_ws/src
  cd my_smart_ws/src
```

2. Clone the robot repo and the SLAM repo
   ```sh
   git clone https://github.com/wvu-irl/smart-2
   ```

4. Build the repository
   ``` sh
   cd ..
   catkin build
   ```


<p align="right">(<a href="#top">back to top</a>)</p>

## Usage
This repository contains for running the SMART robot both in a ROS only system or . This code also support multi robot operation. 

1. The following launch file is used to start the base hardware for each robot 
``` sh
roslaunch smart2_bring_up smart_2.launch
```

### Matlab
Examples of interfacing ROS and matlab and example code for this robot are under the smart2_ros_brige folder. 

### ROS

### Multi Robot
Namespaces changes under the launch smart2_bring_up/smart2.launch file, ca_driver/config/default.yaml file, and smart_board/hw_interface/config/SMART_Board_Serial_launch_params.yaml are needed for running multiple robots at the same time (TODO, add args to change in only one place)

For running multiple robots under a single rosmaster environment variables are needed to be set.  (see:http://wiki.ros.org/ROS/EnvironmentVariables)
ROS_MASTER_URI must be set to the ip of the computer running as master and in each robot computer the ROS_HOSTNAME to the ip address of the local network they are connected to.

## Roadmap
- ( ) Fix the namespaces for changing the robot name only in one place for multirobot use (currently need to change inside hardware workspace and other places).
- ( ) Simplify the process of setting multiple robots in the network. 
- ( ) Integrate with ROS Navigation and ROS localization package. 
- ( ) FIx hardware_interface package name and plugins to not yield warning message. 

<p align="right">(<a href="#top">back to top</a>)</p>


## Contributing
Contribution to the project are greatly appreciated. 
You can contribute by forking the repository or by creating an issue with suggestions/

1. Fork the Project
2. Create your Branch (`git checkout -b MyBranch`)
3. Commit your Changes (`git commit -m 'Added some new feature'`)
4. Push to the Branch (`git push origin MyBranch`)
5. Open a Pull Request
<p align="right">(<a href="#top">back to top</a>)</p>

## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>

## Contact
Chris Tatsch ca0055@mix.wvu.edu

Yu Gu yu.gu@mail.wvu.edu

<p align="right">(<a href="#top">back to top</a>)</p>

## Acknowledgements
<p align="right">(<a href="#top">back to top</a>)</p>


[license-url]: https://github.com/wvu-robotics/LICENSE.txt
[license-shield]: https://img.shields.io/github/license/wvu-irl/smart-2




