# 个人信息 | Zippen's Info    

[头像图片]

a mobile robot maker, especially the field of automatic control and planning   
keywords:    
qq: huang_zhipeng@foxmail.com   


# 研究领域 | Field of Research
自动驾驶（融合感知与规划、自动控制） | self-driving system(integrated perception and planning,  motion control)    
仿生与特种机器人 | biologically inspired and specialized robot design and control    

# 成果发表 | Publications

## 论文 | papers：
in preparation

## 专利 | patents:
```markdown
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
```       

# 项目经历 | Robot Projects Development
Robot projects practice, recorded in chronological order 

## 4 基于ROS的“空地协同”机器人系统 | ROS-based "Air-ground Coordination" Robot System     (2019.04-2019.05)
```markdown
简介 | introduction
    项目主要为全向移动机器人与无人机之间配合，并沿着设有障碍物的路径行进到终点。无人机通过摄像头识别移动机器人顶部的二维码，以实现跟随运动；移动机器人通过Intel RealSense采集RGB图像，矫正移动机器人的姿态，通过分析深度图，判断前方障碍物的大小与位置，实现避障。
   The project worked with an omnidirectional mobile robot and a quadrotor, and travels along the path with obstacles to the end. The quadrotor recognizes the QR code on the top of the mobile robot through the camera to achieve the following action; the mobile robot acquires the RGB image through Intel RealSense, corrects the posture of the mobile robot, and analyzes the depth map to determine the size and position of the obstacle in front, thereby avoiding barrier.
[展示图片] 
[Link](url)
```

## 4 基于二维码定位抓取的自主移动机器人 | Autonomous Mobile Robot Based on QR Code Positioning and Grabbing     (2018.09-2018.12)
```markdown
简介 | introduction
    项目主要设计一款搭载四自由度机械臂的全向移动平台，并通过ZED双目摄像头识别附有二维码的物体的空间位姿，完成对目标物的移动、定位与抓取操作。
    The project designed an omnidirectional mobile platform equipped with a four-degree-of-freedom manipulator, and identifies the spatial pose of the object with the QR code through the ZED stereo camera to complete the moving, positioning and grasping operation of the target.
[展示图片] 
[Link](url)
```

## 4 六足无人平台动力分析与电源管理 | Intelligent Follow Logistics Vehicle     (2017.09-2018.04)
```markdown
简介 | introduction
    项目主要研究六足机器人。主要负责机器人动力特性分析，并设计电源管理系统。
    The project focused on hexapod robots. Mainly responsible for the analysis of robot dynamic characteristics and design of power management system.
[展示图片] 
[Link](url)
```


## 4 基于手势控制的智能跟随物流车 | Intelligent Follow Logistics Vehicle     (2016.10-2017.06)

```markdown
简介 | introduction
    项目主要研究设计兼具手势控制、智能跟随功能的大载重物流车。基于kinect采集人体姿态信息，将之转化为机器人运动控制信息，实现机器人智能跟随。
    The project designed large-load logistics vehicles with gesture control and intelligent following functions. Based on kinect camera, the human body posture information is collected and converted into robot motion control information to realize robot intelligent following.
[展示图片] 
[Link](url)
```

## 4 单电机螺旋式驱动机器人 | Single-motor Spiral-driven Robot     (2016.06-2016.09)

```markdown
简介 | introduction
    项目主要研制单电机螺旋式驱动的波浪式机器人，通过单电机驱动空间螺旋线机构绕轴线旋转，带动多级串联连杆机构呈“波浪式”运动。
    The project developed a single-motor spiral-driven wave-type robot, which rotates around the axis by a single-motor-driven space spiral mechanism, and drives the multi-stage series linkage mechanism to be “wave-like”.
[展示图片] 
[参考文献 | reference paper]
[Link](url)
```

## 3 重力自平衡机械臂 | Gravity-balance Robot Arm     (2016.04-2016.09)
  
```markdown
简介 | introduction 
    项目主要研究适用于机械臂的负载能力提升装置，采用内嵌式机构平衡机械臂自身重力，并设计应用该辅助装置的小型机械臂。
    The project studied the load capacity enhancement device applicable to the robot arm, and uses an in-line mechanism to balance the gravity of the arm itself, and a small robot arm to which the auxiliary device is applied.
[展示图片]
[参考文献 | reference paper]
[Link](url)
```

## 2 仿生四足机器人 | Bionic Quadruped Robot     (2015.08-2015.12)
  
```markdown
简介 | introduction 
    项目主要设计一种高速、低惯量、八连杆单自由度仿生腿，以此作为机器人后腿；采用六连杆机构设计机器人前腿，基于此设计仿生腿的仿生四足机器人。
    The project designed a high-speed, low-inertia, eight-link single-degree-of-freedom bionic leg, which is used as the rear leg of the robot. The six-link mechanism is used to design the front leg of the robot. Based on this, the bionic quadruped robot with bionic legs is designed.
[展示图片]
[参考文献 | reference paper]
[Link](url)
```

## 1 管道检测机器人 | Pipe Inspection Robot      (2014.11-2015.05)
  
```markdown
简介 | introduction
    项目主要设计一款能够适用于内管道的可主/被动变形的管道爬行机器人，可通过机构主动调节机器人外围半径，亦可通过弹性元件被动调节机器人外围半径。设计前后对称、总共三（多）节模块化机体，并在机器人前端与后端安装探伤检测仪器，合成运动性能好、扩展性强的管道爬行机器人。
    The project  designed a pipeline inspection robot that can be applied to the inner pipeline with active/passive deformation. The mechanism can actively adjust the peripheral radius of the robot, and the outer radius of the robot can be passively adjusted by the elastic component. It is symmetrical and modularized, and the detection instrument is installed at the front end and the rear end of the robot to synthesize a pipe crawling robot with good performance and extensibility.
[展示图片] 
[视频展示 | result show](http://v.youku.com/v_show/id_XMTI3OTE0MTk5Mg==.html)
```


# 项目开发过程中的积累 | Accumulation in the Process of Robot Development
introduction  

## 第三方库 | the Third-party Libraries 
```markdown
### introduction opensource kit

### 开发平台 | Development Platform
[展示图片] 
[Link](url)

### 开源软件 | Open Source Software
some words
```

# 阅读的优秀文章 | Excellent Papers I've Read
## 规划与避障 | path planning and collision avoidance
```markdown
1 paper
2 paper
[Link](url)
```
## 控制算法 | control algorithms
```markdown
introduction 
[Link](url)
```
## 感知算法 | perception algorithms
```markdown
introduction 
[Link](url)
```

## 定位算法 | location algorithms
```markdown
introduction 
[Link](url)
```

# 优秀的网站 | Excellent WEB I've Seen
introduction  

## Person Main Pages 
```markdown
### introduction opensource kit
yangzhiping's main page:[Link](https://www.yangzhiping.com/info/blogger.html)
### Development Platform
[展示图片] 
[Link](https://www.yangzhiping.com/info/blogger.html)

### Open Source Software
some words
```


# 爱好 | Likes
introduction 












You can use the [editor on GitHub](https://github.com/Zippen-Huang/Zippen-Huang.github.io/edit/master/README.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/Zippen-Huang/Zippen-Huang.github.io/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://help.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.
