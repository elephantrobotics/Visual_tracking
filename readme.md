# 视觉跟踪使用说明

## 2025.4.1更新说明

#### 1.适配了最新的固件

    pymycobot 3.9.1 #可通过pip更新
    mycobot280 atom_v7.2 #可在官网mystudio软件中更新
    mycobot320 pico_v1.5 #可在官网mystudio软件中更新

**现在无需烧录专用固件，在官网更新最新固件即可**

#### 2.新增视觉跟踪模式set_vision_mode(1-open 0-close)

#### 3.修改了运动的初始姿态

#### 4.现在初始化串口时需要手动选择机型，如：

    mc = MyCobot280("COM32")  # 需要手动设置端口及型号
    mc = MyCobot320("COM32")  # 需要手动设置端口及型号

## 2024.11.11更新说明

**修复了该识别库在树莓派系统中无法正常打开相机的问题**

#### 1.删除了cv2.imshow(“输入任意键退出”，frame)中的中文输出，该使用方式会导致linux系统中相机图像无法正常显示

#### 2.请根据操作系统中修改uvc_camera.py文件中的配置参数
    self.cap = cv2.VideoCapture(self.cam_index) #linux
    self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW) #windows

## 2024.10.22更新说明
#### 1.程序开头处添加了根据机器类型设置观测点offset的功能，该功能用于区分不同机器的零位，使得用户无需手动区分不同机型的观测姿态

    type = mc.get_system_version()
    offset_j5 = 0
    if type > 2:
        offset_j5 = -90
        print("280")
    else:
        print("320")

#### 可以看到观测姿态增加了offset

    self.origin_mycbot_horizontal = [42.36, -35.85, -52.91, 88.59, -42.62+offset_j5, 0.0]
    self.origin_mycbot_level = [-90, 5, -104, 14, 90 + offset_j5, 0]

#### 2.优化了视觉跟踪的初始姿态，新的观测姿态不再临近奇异点

#### 3.优化了手眼标定

1)优化了手眼标定接口中的识别逻辑，从识别三次修改为了识别五次

    tbe = np.vstack([tbe1, tbe2, tbe3, tbe4, tbe5])
    Mc = np.vstack([Mc1, Mc2, Mc3, Mc4, Mc5])

2)增加了手眼矩阵的**验证**环节，现在标定结束后会再识别一次用于验证

    for i in range(1, r):
        for j in range(3):
            err = abs(pos[i][j] - pos[0][j])
            if(err > 10):
                state = False
                print("matrix error")

3）在手眼矩阵中设置了300mm的**工具偏移**，用于跟踪时保持相机和物体的距离，无需再手动对target_coords进行偏移

    self.IDENTIFY_LEN = 300 #to keep identify length
    tce[2] -= self.IDENTIFY_LEN #用于保持识别距离

#### 4.新增ids识别功能，在视觉跟踪时会对编号为0的stag码进行跟踪，识别到编号1的stag时会回到初始观测姿态

    while 1:
        _ ,ids = self.stag_identify()
        if ids[0] == 0:
            ml.send_coords(target_coords, 30)  # 机械臂移动到二维码前方
            # time.sleep(0.5)
        elif ids[0] == 1:
            ml.send_angles(self.origin_mycbot_horizontal, 50)  # 移动到观测点


**5.优化了部分代码逻辑，对重复调用的部分进行了封装**

    pose, tbe, Mc, state = self.Matrix_identify(ml)


### 1 使用前准备
#### 1.1 Python依赖库
使用pip安装以下Python库

    pip install stag-python

    pip install opencv-python

    pip install scipy

    pip install numpy

    pip install pymycobot

**以下版本确认可用**

windows

    stag-python         1.0.1
    opencv-python       4.8.1.78
    scipy               1.11.4
    numpy               1.26.2
    pymycobot           3.6.0

linux

    stag-python         1.0.2
    opencv-python       4.10.0.84
    scipy               1.10.1
    numpy               1.24.2
    pymycobot           3.6.0


#### 1.2 Stag码
本文使用stag码用作二维码跟踪
 <div align=center><img src="resources\stag.png" width="20%" ></div>

**建议使用彩印，黑白打印识别率较低**

下载地址如下：
    https://drive.google.com/drive/folders/0ByNTNYCAhWbILXd2SE5FY1c3WXM?resourcekey=0-nWeENtNZql2j9AF32Ud8sQ

    stag码的左上角为编号，使用opencv的stag识别库可以识别该编号，你可以为不同编号设计不同的行为逻辑，比如00设为位置跟踪，01设为回到观测点，02设为姿态跟踪。

#### 1.2 相机初始化
在本项目的camera_detect.py文件中定义了名为camera_detect的视觉识别类

    class camera_detect:
        #Camera parameter initialize
        def __init__(self, camera_id, marker_size, mtx, dist):

其初始化的四个参数分别为：

    camera_id 相机编号（范围一般是0~10，默认为0）
    marker_size Stag码的边长（mm）
    mtx, dist 相机参数

在程序中已经给了初始化的示例，用户仅需修改camera_id和marker_size即可

    camera_params = np.load("camera_params.npz")  # 相机配置文件
    mtx, dist = camera_params["mtx"], camera_params["dist"]
    m = camera_detect(0, 32, mtx, dist)

camera_id的测试方法如下：
从0开始设置camera_id，使用类中的camera_open_loop函数打开对应编号的相机，从弹窗中观测视野是否正确

    m = camera_detect(0, 32, mtx, dist)
    m.camera_open_loop()
如果无画面或开启了错误的相机，修改camera_id重复上述步骤

    m = camera_detect(1, 32, mtx, dist)
    m.camera_open_loop()

**注意：如果你开启相机的速度非常慢（>10s），请根据操作系统修改对应配置**

打开uvc_camera.py文件，找到下面这行语句

    def capture(self):
        self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW) #windows
如果是非windows系统，修改为

    def capture(self):
        self.cap = cv2.VideoCapture(self.cam_index)

### 2 手眼标定
#### 2.1 手眼矩阵原理

#### 手眼标定是视觉跟踪必不可少的一环，其作用是求得机械臂坐标系（手）与相机坐标系（眼）之间的相对关系，我们把这种相对关系用一个4*4的手眼矩阵来表示，具体原理可以参考以下链接：

https://blog.csdn.net/weixin_45844515/article/details/125571550

#### 2.2 手眼标定方法

**标定的流程及具体效果可见视频**

<video controls src="手眼标定.mp4" title="Title"></video>

在1.2节中我们完成了相机的初始化，将相机装配到机械臂上（一般装配在机械臂末端），连接需要控制的机械臂

设置机械臂的端口号，调用camera_detect中的手眼标定接口

    mc = MyCobot("COM8")  # 设置端口
    m.Eyes_in_hand_calibration(mc)  #手眼标定

手眼标定支持对工具端标定，可以按照以下形式设置工具坐标系：

    tool_len = 20
    mc.set_tool_reference([0, 0, tool_len, 0, 0, 0])
    mc.set_end_type(1)

此时机械臂会先运动到观测姿态

    self.origin_mycbot_level = [-90, 5, -104, 14, 90 + offset_j5, 0]
    
    def Eyes_in_hand_calibration(self, ml):
        ml.send_angles(self.origin_mycbot_level, 50)  # 移动到观测点

#### 用户可自由修改观测点位，比如旋转6关节令相机处于更适合的位置

运动到观测姿态后，终端会弹出以下提示，将stag码置于相机视野内，输入任意键即可继续识别

    input("make sure camera can observe the stag, enter any key quit")

若相机识别到stag码，则会自动进入下一步识别，机械臂移动并捕捉机械臂和相机的位置信息

在移动并识别的过程中，程序会自动对照当前储存的手眼矩阵文件，如果识别结果趋于一致，则会弹出**Calibration Complete EyesInHand_matrix**表示标定完毕
#### 如果你希望标定一组新的手眼矩阵参数，可在标定前删除EyesInHand_matrix.json文件！

当为弹出**matrix error**的提示，表示需要进行手眼标定，终端会弹出提示，输入任意键后机械臂会放松，根据提示移动机械臂末端至贴紧二维码（详细可见视频）

    input("Move the end of the robot arm to the calibration point, press any key to release servo")

贴紧后根据提示，输入任意键完成手眼标定

    input("focus servo and get current coords")

此时会打印**save successe, wait to verify**信息，程序将自动进入验证环节


**验证环节**: 完成标定后机械臂会回到手眼标定的观测姿态，此时会重复识别并验证标定参数是否正确，请保证stag码仍然在相机视野内，此时若显示"matrix error"则表示验证未通过，需用重新运行标定程序！

**标定完成后手眼矩阵会以EyesInHand_matrix.json的形式储存，标定成功后无需重复操作！**


#### 注意：手眼标定可能会由于操作不当、机器虚位等原因出现误差，当视觉跟踪效果不好时，需要重新手眼标定!

### 3 视觉跟踪

**具体效果可见视频**

<video controls src="视觉跟踪.mp4" title="
"></video>

完成了手眼标定后，使用vision_trace_loop接口即可以进行视觉跟踪

    m.vision_trace_loop(mc) #视觉跟踪

调用该函数后会机械臂会运动到观测点

    self.origin_mycbot_horizontal = [42.36, -35.85, -52.91, 88.59, -42.62+offset_j5, 0.0]
    ml.send_angles(self.origin_mycbot_horizontal, 50)  # 移动到观测点

用户可自行修改观测点

    def vision_trace_loop(self, ml):
        mc.set_fresh_mode(1)
        time.sleep(1)

        ml.send_angles(self.origin_mycbot_horizontal, 50)  # 移动到观测点
        self.wait()
        origin = ml.get_coords()
        while origin is None:
            origin = ml.get_coords()
        time.sleep(1)
        while 1:
            _ ,ids = self.stag_identify()
            if ids[0] == 0:
                self.camera.update_frame()  # 刷新相机界面
                frame = self.camera.color_frame()  # 获取当前帧
                cv2.imshow("Enter any key to exit", frame)

                target_coords,_ = self.stag_robot_identify(ml)
                self.coord_limit(target_coords)
                print(target_coords)
                for i in range(3):
                    target_coords[i+3] = origin[i+3]
                ml.send_coords(target_coords, 30)  # 机械臂移动到二维码前方
                # time.sleep(0.5)
            elif ids[0] == 1:
                ml.send_angles(self.origin_mycbot_horizontal, 50)  # 移动到观测点

**_ ,ids = self.stag_identify()**

此处识别了stag码的ids编号，并针对不同的编号设置了不同的行为逻辑，其中0表示跟踪，1表示回到观测姿态

**target_coords = self.stag_robot_identify(ml)**

target_coords是计算出的基于机械臂坐标系的stag码坐标

由于我们在手眼矩阵中添加了300mm的**工具偏移**，此时计算得到的target_coords与物体的实际位置存在300mm的偏差，该偏差有助于保持相机的观测距离

**self.coord_limit(target_coords)**

这里限制了机械臂的活动范围，用户可自行修改运动范围

### 4 机械臂运动效果调优
**使用stag跟踪专用接口效果更佳**

    set_vision_mode(1)  #开启视觉跟踪模式

#### 版本要求

**280：**

    在官网mystudio中烧录atom_v7.2以上固件

**320：**

    在官网mystudio中烧录pico_v1.5以上固件

#### 该模式会限制机械臂的姿态偏转，在刷新模式下使用send_coords接口会筛选掉角度偏差超过90°的相邻坐标

