### 如何运行代码 How to run the program

本项目的代码分为`main.py`和`server_code.py`两部分。

`main.py`在mac端运行，使用Mediapipe和OpenCV进行手势识别及追踪，运行时需要先确保机器人端程序开始运行且socket连接成功

`server_code.py`在树莓派上运行，编写和运行时需要先连接机器人网络：

- 名称：HawkBot
- 密码：hawkbot123

再ssh至树莓派端：

- IP地址：`pi@10.42.0.1`（若使用配置好的Linux系统进行操作，可直接输入`sshrobot`指令）
- 密码：hawkbot

`server_code.py`文件在树莓派的catkin_ws中的hawkbot package以node形式存在，有独立的launch文件，因此运行时可以直接使用`roslaunch hawkbot server_code.launch`