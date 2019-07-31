# 个人信息 | Zippen's Info    

* AM: a (mobile) robot designer and maker      
* E-mail: huang_zhipeng@foxmail.com   

# 研究领域 | Field of Research
 自动驾驶（融合感知与规划、自动控制） | self-driving system(integrated perception and planning,  motion control)    
 仿生与特种机器人 | biologically inspired and specialized robot design and control    

# 成果发表 | Publications

## 论文 | papers
（in preparation）

## 专利 | patents
[1] 一种增力机械手爪. (201620043599.1).    
[2] 一种可实用于内、外管道爬行机器人. (201521050698.4).    
[3] 一种新型机械臂. (201621174783.6).    
[4] 简易玉米脱粒机. (201711319375.4).    
[5] 玉米脱粒机. (201711321004.X).    
[6] 楼宇环境新型文件配送系统. (201910119506.7).     
[7] 基于视觉的二维码定位抓取机器人系统算法. (201910011866.5).    
[8] 一种多分裂高压导线断股修补辅助装置及其使用方法. (201910149941.4).     
[9] 一种应用C形腿的六足机器人动力特性分析方法. (201910300123.X).     
[10]基于SMA驱动方式的新型跳跃机器人. (2019103294229.8).        

**************************************************************************************************************************
# 项目经历 | Robot Projects Development
Robot projects practice, recorded in chronological order.    

## 9 面向室外场景的中型移动机器人系统 | Medium-sized Mobile Robot System for Outdoor Scenes
**简介 | Introduction**  (2017.11 - 2020.06)    
&emsp;&emsp;项目主要研制面向室内外半结构化环境下的中型自主移动机器人软硬件系统，旨在为物流派送、园区巡检、市区清洁等应用场景提供智能移动平台理论和技术支撑。在基于ROS机器人系统平台，集成感知系统、定位系统、规划系统与控制系统，实现给定目标点的情况下，机器人能够在室内外自主导航至目的地。    
&emsp;&emsp;The project develops a medium-sized autonomous mobile robot software and hardware system for indoor and outdoor semi-structured environments, aiming to provide intelligent mobile platform theory and technical support for application scenarios such as logistics dispatch, park inspection and urban cleaning. Based on the ROS(robot operating system) platform, integrated perception system, location system, planning and control system, the robot can navigate to the destination autonomously given the target point.  
 ![](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E6%99%BA%E8%83%BD%E7%89%A9%E6%B5%81%E8%BD%A6.jpg)   

## 8 基于ROS的“空地协同”机器人系统 | ROS-based "Air-ground Coordination" Robot System     
**简介 | Introduction**  (2019.04 - 2019.05)    
&emsp;&emsp;项目主要为全向移动机器人与无人机之间配合，并沿着设有障碍物的路径行进到终点。无人机通过摄像头识别移动机器人顶部的二维码，以实现跟随运动；移动机器人通过Intel RealSense采集RGB图像，矫正移动机器人的姿态，通过分析深度图，判断前方障碍物的大小与位置，实现避障。    
&emsp;&emsp;The project worked with an omnidirectional mobile robot and a quadrotor, and travels along the path with obstacles to the end. The quadrotor recognizes the QR code on the top of the mobile robot through the camera to achieve the following action; the mobile robot acquires the RGB image through Intel RealSense, corrects the posture of the mobile robot, and analyzes the depth map to determine the size and position of the obstacle in front, thereby avoiding barrier.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E7%A9%BA%E5%9C%B0%E5%8D%8F%E5%90%8C%E6%9C%BA%E5%99%A8%E4%BA%BA.jpg)   

## 7 基于二维码定位抓取的自主移动机器人 | Autonomous Mobile Robot Based on QR Code Positioning and Grabbing   
**简介 | Introduction**  (2018.09 - 2018.12)    
&emsp;&emsp;项目主要设计一款搭载四自由度机械臂的全向移动平台，并通过ZED双目摄像头识别附有二维码的物体的空间位姿，完成对目标物的移动、定位与抓取操作。    
&emsp;&emsp;The project designed an omnidirectional mobile platform equipped with a four-degree-of-freedom manipulator, and identifies the spatial pose of the object with the QR code through the ZED stereo camera to complete the moving, positioning and grasping operation of the target.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E7%A7%BB%E5%8A%A8%E5%AE%9A%E4%BD%8D%E6%8A%93%E5%8F%96%E6%9C%BA%E5%99%A8%E4%BA%BA.jpg)   

## 6 六足无人平台动力分析与电源管理 | Intelligent Follow Logistics Vehicle    
**简介 | Introduction**  (2017.09 - 2018.04)    
&emsp;&emsp;项目主要研究六足机器人。主要负责机器人动力特性分析，并设计电源管理系统。    
&emsp;&emsp;The project focused on hexapod robots. Mainly responsible for the analysis of robot dynamic characteristics and design of power management system.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E5%85%AD%E8%B6%B3%E6%9C%BA%E5%99%A8%E4%BA%BA.jpg)   

## 5 基于手势控制的智能跟随物流车 | Intelligent Follow Logistics Vehicle    
**简介 | Introduction**  (2016.10 - 2017.06)    
&emsp;&emsp;项目主要研究设计兼具手势控制、智能跟随功能的大载重物流车。基于kinect采集人体姿态信息，将之转化为机器人运动控制信息，实现机器人智能跟随。    
&emsp;&emsp;The project designed large-load logistics vehicles with gesture control and intelligent following functions. Based on kinect camera, the human body posture information is collected and converted into robot motion control information to realize robot intelligent following.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E6%99%BA%E8%83%BD%E8%B7%9F%E9%9A%8F%E7%89%A9%E6%B5%81%E8%BD%A6.jpg)   

## 4 单电机螺旋式驱动机器人 | Single-motor Spiral-driven Robot    
**简介 | Introduction**  (2016.06 - 2016.09)     
&emsp;&emsp;项目主要研制单电机螺旋式驱动的波浪式机器人，通过单电机驱动空间螺旋线机构绕轴线旋转，带动多级串联连杆机构呈“波浪式”运动。    
&emsp;&emsp;The project developed a single-motor spiral-driven wave-type robot, which rotates around the axis by a single-motor-driven space spiral mechanism, and drives the multi-stage series linkage mechanism to be “wave-like”.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E8%9E%BA%E6%97%8B%E5%BC%8F%E9%A9%B1%E5%8A%A8%E6%9C%BA%E5%99%A8%E4%BA%BA.jpg)   

## 3 重力自平衡机械臂 | Gravity-balance Robot Arm    
**简介 | Introduction**  (2016.04 - 2016.09)    
&emsp;&emsp;项目主要研究适用于机械臂的负载能力提升装置，采用内嵌式机构平衡机械臂自身重力，并设计应用该辅助装置的小型机械臂。    
&emsp;&emsp;The project studied the load capacity enhancement device applicable to the robot arm, and uses an in-line mechanism to balance the gravity of the arm itself, and a small robot arm to which the auxiliary device is applied.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E6%9C%BA%E6%A2%B0%E8%87%82.jpg)   

## 2 仿生四足机器人 | Bionic Quadruped Robot  
**简介 | Introduction**  (2015.08 - 2015.12)    
&emsp;&emsp;项目主要设计一种高速、低惯量、八连杆单自由度仿生腿，以此作为机器人后腿；采用六连杆机构设计机器人前腿，基于此设计仿生腿的仿生四足机器人  
&emsp;&emsp;The project designed a high-speed, low-inertia, eight-link single-degree-of-freedom bionic leg, which is used as the rear leg of the robot. The six-link mechanism is used to design the front leg of the robot. Based on this, the bionic quadruped robot with bionic legs is designed.   
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E6%88%98%E7%8B%BC.jpg)   

## 1 管道检测机器人 | Pipe Inspection Robot     
**简介 | introduction**  (2014.11 - 2015.05)    
&emsp;&emsp;项目主要设计一款能够适用于内管道的可主/被动变形的管道爬行机器人，可通过机构主动调节机器人外围半径，亦可通过弹性元件被动调节机器人外围半径。设计前后对称、总共三（多）节模块化机体，并在机器人前端与后端安装探伤检测仪器，合成运动性能好、扩展性强的管道爬行机器人。    
&emsp;&emsp;The project  designed a pipeline inspection robot that can be applied to the inner pipeline with active/passive deformation. The mechanism can actively adjust the peripheral radius of the robot, and the outer radius of the robot can be passively adjusted by the elastic component. It is symmetrical and modularized, and the detection instrument is installed at the front end and the rear end of the robot to synthesize a pipe crawling robot with good performance and extensibility.    
 ![image](https://github.com/Zippen-Huang/Zippen-Huang.github.io/blob/master/01-proj_images/%E7%AE%A1%E9%81%93%E6%8E%A2%E4%BC%A4%E6%9C%BA%E5%99%A8%E4%BA%BA.jpg)   
[视频展示 | result show](http://v.youku.com/v_show/id_XMTI3OTE0MTk5Mg==.html)    

**************************************************************************************************************************

# 机器人项目积累 | Accumulation in the Process of Robot Development
记录在项目开发过程中非常有用的资源   
Document useful resources during project development    

## 1 软件 | Software  
### 1.1 开发平台 | Development Platforms
* ROS
* RVIZ
* Gazabo
* V-REP
* Stage
* player

* Machine Learning: tensorflow, Caffe, Keras, PyTorch, and so on   

### 1.2 开源库 | the Third-party Libraries
* Armadillo：C++下的Matlab替代品
* Eigen3
* OMPL(Open Motion Planning Library)
* Boost
* opencv
* pcl
* matlab robotics toolbox

### 1.3 其他软件 | Others

## 2硬件 | Hardware  
###  2.1 处理器平台 | Processor Platforms
#### 2.1.1 高级处理器　｜　High-level　Processors
* NVIDIA Jetson Family: TK1, TX1, TX2, NANO, Xaiver
* Intel Family: Intel NUC, Galileo, Edison
* Raspberry Pi Series
* ODROID Series
* 华为机器人开发板  HUAWEI HAISI hikey970

#### 2.1.2 低级处理器　｜　Low-level　Processors
* STM32 Series
* Arduino Series
* openCR


###  2.2 传感器 | Sensors
#### 2.2.1 感知类 ｜Perception
##### 视觉传感器 | Camera
* MicroSoft Kinect
* Intel RealSense
* ZED
* 奥比中光  Astra

##### 激光雷达传感器 | Lidar
* Velodyne
* Sick
* Ibeo
* Hokuyo
* Trimble
* 思岚科技  SLAMTEC
* 速腾聚创
* 禾赛科技
* 北醒光子
* 镭神智能

#### 2.1.2 定位类　｜　Location
* razor-imu-9dof
* Ublox GPS

###  2.1.3 驱动及电机 | Actuators(Motors)
* DYNAMIXEL
* maxon motor

###  2.1.4 机器人平台 | Robot that Integrated Hardware and Software
#### Mobile Robots 
* Turtlebot1, Turtlebot2, Turtlebot3    
* HUSKY   
* RACE CAR   
* Autolabor Pro   

#### Robot Arm  
* Dobot
* uArm
* UR5

###  2.1.5 其他硬件 | Others

**************************************************************************************************************************

# 经典文献 | Excellent Papers I've Read
## 1.规划与避障 | path planning and collision avoidance algorithms
### 1.1 动态窗 | DWA(Dynamic Window Approach)
* Fox D, Burgard W, Thrun S. The dynamic window approach to collision avoidance[J]. IEEE Robotics & Automation Magazine, 1997, 4(1): 23-33.   
* Brock O, Khatib O. High-speed navigation using the global dynamic window approach[C]//Proceedings 1999 IEEE International Conference on Robotics and Automation (Cat. No. 99CH36288C). IEEE, 1999, 1: 341-346.   

### 1.2 时变橡皮筋算法 | TEB(Timed-Elastic-Band)
* Rösmann C, Hoffmann F, Bertram T. Integrated online trajectory planning and optimization in distinctive topologies[J]. Robotics and Autonomous Systems, 2017, 88: 142-153.   
* Rösmann C, Hoffmann F, Bertram T. Kinodynamic trajectory optimization and control for car-like robots[C]//2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017: 5681-5686.   
* 算法代码：[teb_local_planner](http://wiki.ros.org/teb_local_planner)  

## 2.控制算法 | control algorithms
### 2.1 模型预测控制器 | MPC(Model Predictive Control)

### 2.2 线性二次型调节器 | LQR(Linear Quadratic Regulator)

### 2.3 滑模控制器 | SMC(sliding mode control)

### 2.4 PID

## 3.感知算法 | perception algorithms
### 3.1 深度学习方向 | Deep Learning
* YOLO
* SSD

## 4.定位算法 | location algorithms

## 5.滤波算法 | Filter algorithms　
### 5.1 卡尔曼滤波 | KF(Kalman Filter)
* EKF(Extend Kalman Filter)
* UKF(Unscented Kalman Filter)

### 5.2 粒子滤波 | PF(Particle Filtering)


## 6.算法缓冲区 | algorithms buffer

**************************************************************************************************************************
# 优秀网站 | Excellent Web I've Seen
Excellent Website Collection

## 1.个人主页 | Personal Main Pages 
* 机械臂和轨迹规划算法相关：[Brian2018](https://blog.csdn.net/libing403/article/details/89344476)    
* NVIDIA JETSON系列开发板及RACE CAR教程网站：[JetsonHacks](https://www.jetsonhacks.com/)

## 2.综合型网站 | Comprehensive Web
* 机器人软硬件及其编程教程汇集：[创客智造](https://www.ncnynl.com/)

## 3.资源搜索型网站 | Resource Search Web


**************************************************************************************************************************
# 爱好 | Hobbies
reading and sports
