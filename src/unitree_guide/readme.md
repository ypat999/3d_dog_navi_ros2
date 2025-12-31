# unitree_guide

#### 介绍
gazebo 强化学习部署
 **实机效果很差，麻烦佬实机验证** 
#### 软件架构
软件架构说明


#### 安装教程

1.  将代码clone到ros工作目录/src下
目录结构为 ros工作目录/src/unitree_guide
2.  cd ros工作目录 catkin_make
3.  source ./devel/setup.bash
4.  cd src/unitree_guide
5.  chmod 777 ./auto.sh
6.  修改ros工作目录/src/unitree_guide/unitree_guide/unitree_guide/src/FSMState_RL_test.cpp中的model_path为unitree_guide/logs中的模型绝对路径
7.  修改ros工作目录/src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt中的libtorch路径以及CMAKE_CUDA_COMPILER路径
8.  ./auto.sh    #可以在auto中修改robot
9.  ../../devel/lib/unitree_guide/junior_ctrl
10. 一定要连接手柄，否则会发生core dumped，按2进入站立，6进入rl，然后手柄遥控
####也可以将unitree_guide/ 下的文件全部剪切到/src下防止目录太深
![plane](logs/plane.gif)

![stair](logs/stair.gif)

有两个版本的模型，平地以及楼梯，楼梯自旋转不行，平地原地旋转可以，有MP4文件，logs目录下