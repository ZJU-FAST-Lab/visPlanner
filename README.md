# visPlanner


**visPlanner** is a visibility-aware trajectory planning framework that can deal with  visibility of the target in aerial tracking.

**Authors**: Qianhao Wang, Yuman Gao, Jialin Ji, Chao Xu and [Fei Gao](https://ustfei.com/) from the [ZJU Fast Lab](http://zju-fast.com/). 

**Paper**: [Visibility-aware Trajectory Optimization with Application to Aerial Tracking](https://arxiv.org/abs/2103.06742),  Accepted in IEEE International Workshop on Intelligent Robots and Systems (__IROS 2021__).


**Video Links**: [youtube](https://www.youtube.com/watch?v=PhhrOBx54YY) or [bilibili](https://www.bilibili.com/video/BV1vh411Q7G9/)


[NOTE] remember to change the CUDA option of **src/uav_simulator/local_sensing/CMakeLists.txt** based on your GPU.

>Run the visualization:
```
git clone https://github.com/ZJU-FAST-Lab/visPlanner.git
cd visPlanner
catkin_make
source devel/setup.zsh
roslaunch ego_planner rviz.launch
```
>Run the visPlanner:
```
roslaunch ego_planner tracking.launch
```
>Triger the target drone with the ``3D Nav Goal`` in rviz to fly and the other drone will track the target drone:

<p align="center">
    <img src="figs/rviz.gif" width="700"/>
</p>

We made some improvements based on [Ego-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) and  [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)   to make the trajectory optimization in the framework work better.

