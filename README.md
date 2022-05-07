# Python-Trajectory-Tracking-Control-for-UAV
单无人机对螺旋轨迹跟踪的实物实验  实验视频已上传B站：https://www.bilibili.com/video/BV1fL4y1t7bA?spm_id_from=333.999.0.0  无人机用的是bebop，软件框架是ROS，语言为Python，硬件不同跑不起来，不过可以参考代码中控制逻辑，希望能有些帮助。  控制方法是对串级PID控制，外环是位置环，内环是速度环  终端运行bebop_ctrl.py，在弹出的qt界面中进行操作，控制bebop无人机  无人机的控制代码在trajectory_tracking.py中。
