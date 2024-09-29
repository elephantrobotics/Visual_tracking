# 视觉跟踪使用说明
### 1 使用前准备
#### 1.1 Python依赖库
使用pip安装以下Python库

    pip install stag-python

    pip install opencv-python

    pip install scipy

    pip install numpy

    pip install pymycobot


#### 1.2 Stag码
本文使用stag码用作二维码跟踪
 <div align=center><img src="resources\stag.png" width="20%" ></div>
**建议使用彩印，黑白打印识别率较低**

下载地址如下：
    https://drive.google.com/drive/folders/0ByNTNYCAhWbILXd2SE5FY1c3WXM?resourcekey=0-nWeENtNZql2j9AF32Ud8sQ

    stag码的左上角为编号，使用opencv的stag识别库可以识别该编号，你可以为不同编号设计不同的行为逻辑，比如00设为位置跟踪，01设为姿态跟踪，02设为回到观测点。

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

    self.origin_mycbot_level = [0, 5, -104, 14, 0, 0]
    def Eyes_in_hand_calibration(self, ml):
        ml.send_angles(self.origin_mycbot_level, 50)  # 移动到观测点

#### 用户可自由修改观测点位，比如旋转6关节令相机处于更适合的位置

运动到观测姿态后，终端会弹出以下提示，将stag码置于相机视野内，输入任意键即可继续识别

    input("make sure camera can observe the stag, enter any key quit")

若相机识别到stag码，则会自动进入下一步识别，机械臂移动并捕捉机械臂和相机的位置信息

完毕后终端会弹出提示，输入任意键后机械臂会放松，根据提示移动机械臂末端至贴紧二维码（详细可见视频）

    input("Move the end of the robot arm to the calibration point, press any key to release servo")

贴紧后根据提示，输入任意键完成手眼标定

    input("focus servo and get current coords")

此时会打印EyesInHand_matrix信息，视为标定完成

**标定完成后手眼矩阵会以EyesInHand_matrix.json的形式储存，标定成功后无需重复操作！**

### 注意：手眼标定可能会由于操作不当、机器虚位等原因出现误差，当视觉跟踪效果不好时，需要重新手眼标定

### 3 视觉跟踪

**具体效果可见视频**

<video controls src="视觉跟踪.mp4" title="
"></video>


完成了手眼标定后，使用vision_trace_loop接口即可以进行视觉跟踪

    m.vision_trace_loop(mc) #视觉跟踪

调用该函数后会机械臂会运动到观测点

    self.origin_mycbot_horizontal = [0,60,-60,0,0,0]
    ml.send_angles(self.origin_mycbot_horizontal, 50)  # 移动到观测点

用户可自行修改观测点

    def vision_trace_loop(self, ml):
        mc.set_fresh_mode(1)
        time.sleep(1)

        ml.send_angles(self.origin_mycbot_horizontal, 50)  # 移动到观测点
        self.wait()
        time.sleep(1)
        origin = ml.get_coords()
        while origin is None:
            origin = ml.get_coords()
        while 1:
            self.camera.update_frame()  # 刷新相机界面
            frame = self.camera.color_frame()  # 获取当前帧
            cv2.imshow("按下键盘任意键退出", frame)

            target_coords = self.stag_robot_identify(ml)
            target_coords[0] -= 300
            self.coord_limit(target_coords)

**target_coords = self.stag_robot_identify(ml)**

target_coords是计算出的基于机械臂坐标系的stag码坐标

**target_coords[0] -= 300**

这里对坐标的x轴减去了300mm，意味着机械臂将运动到距离stag码300mm的位置，如果修改了观测点的姿态，此处也需同步修改！

**self.coord_limit(target_coords)**

这里限制了机械臂的活动范围，用户可自行修改运动范围

### 4 机械臂运动效果调优
**使用stag跟踪专用固件效果更佳**

**280：**

mycobot280_atom0926_vision.bin

**320：**

mycobot320_atom0926_vision.bin

#### 该固件会限制机械臂的姿态偏转，在刷新模式下使用send_coords接口会筛选掉角度偏差超过90°的相邻坐标

