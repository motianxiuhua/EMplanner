# Carla-CarlaRosBridge-Ros工具链实现EMplanner超车

视频演示链接：

[Carla超车](https://www.bilibili.com/video/BV1Pn4y1R7rN/?vd_source=ad600e50c7e20ee4e5d7bb36ee34ce25)

超车构建了虚拟障碍物，但是虚拟障碍物的构建需要依赖于轨迹预测，这里没有接入轨迹预测，只采用了匀速预测的逻辑，所以超车需要调整一些参数，视频中的参数并没有调整好，导致最后撞车了。

 因为我的carla版本配置过其他的版本，导致vscode调试无法正确识别，就只是调试用不了，难以定位问题，并且这个问题并不是很重要，现在并不想去调，如果后面的人要用的话，请注意。

 本代码结构化可读性比较强,主要参考了以下两份代码，。

[carla_ros_bridge_pnc](https://github.com/czjaixuexi/carla_ros_bridge_pnc)

 [driving-planning](https://github.com/zhaokun506/driving-planning)

## 1.安装依赖：

```c++
Ubuntu 18.04/20.04
ROS Melodic/Noetic
Eigen
cmake >= 3.5
python3.X
Carla simulator (0.9.11)
```
安装配置手册：[https://carla.readthedocs.io/en/latest/start_quickstart/](https://carla.readthedocs.io/en/latest/start_quickstart/)
ROS bridge for CARLA simulator(0.9.11)

安装配置手册：[https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/)

Carla scenario_runner(0.9.11)

​ 安装配置手册：[https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/](https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/)

## 配置方式

1.创建src文件，放置功能包源码：

```c++
   mkdir -p ~/catkin_ws/src
```
2.进入src文件夹
```c++
   cd ~/catkin_ws/src
```
3.将功能包复制到的src目录下
4.初始化文件夹

```c++
   catkin_init_workspace
```
5.编译工作空间
```c++
   catkin build
#catkin build -DCMAKE_BUILD_TYPE=Debug  #for debug
```
## 启动Planning

终端1：启动carla

```plain
cd path/to/carla/root
./CarlaUE4.sh
```

终端2：启动Planning结点

```plain
source devel/setup.bash
```

EM Planner

```plain
roslaunch planning planning_demo.launch 
```

终端3：启动scenario runner

scenario_runner-0.9.11目录下，代码在[scenario_runner](https://github.com/czjaixuexi/scenario_runner)

```plain
python scenario_runner.py --scenario Avoid_and_Follow_1 --waitForEgo  
```

