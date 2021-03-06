**F1/10 Integration for Cooperative Platooning**

In this repository, the F1/10 Autonomous Racing was changed in order to support a cooperative platooning of three cars, adding our own control algorithm.

**Requisites**

- ROS Melodic
- Gazebo 9

Consult http://wiki.ros.org/melodic/Installation/Ubuntu for installing ROS Melodic


**1. Installation**

```
1. $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
2. $ sudo apt update
3. $ sudo apt install ros-melodic-desktop-full
4. $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
5. $ source ~/.bashrc
```

Once you've installed ROS Melodic you need to create a workspace

```
$ cd ~/<location>
$ mkdir <directory_name>/src
```

**A. Initialize your workspace**

`$ catkin_init_workspace`

In the same terminal

```
$ cd ..
$ catkin_make install
```

Source the setup.bash file from inside level(optional)

In the case you don't proceed this step, you need to source the directory every time

```
$ echo "source ~/Desktop/<directory_name>/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

If you already have installed ROS Melodic, you should install the following dependecies

**B. Install repositories**

```
1. $ cd sims_ws/src
2. $ git clone https://github.com/wjwwood/serial.git
3. $ git clone https://github.com/mit-racecar/racecar.git
4. $ git clone https://github.com/mlab-upenn/racecar-simulator.git
5. $ git clone https://github.com/mit-racecar/vesc.git
6. $ git clone https://github.com/ros-drivers/ackermann_msgs.git
7. $ git clone https://github.com/mlab-upenn/f1_10_sim.git
```

**2. Simulation**

Now, you should be able to run the simulation, in order to do it you must execute the following steps

```
$ roscore
$ roslaunch race test_f1_10.launch
$ rosrun race f1_10_control.py
```

Second and third command must be executed in different terminals
