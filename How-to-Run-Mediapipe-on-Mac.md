## 如何在Mac上运行Mediapipe提供的手部追踪代码

1. 使用homebrew在Mac上下载OpenCV：

   由于网络原因，在国内下载homebrew无法使用官方镜像，可以使用中科大/清华等学校或机构提供的镜像

   ```shell
   /bin/zsh -c "$(curl -fsSL [https://gitee.com/cunkai/HomebrewCN/raw/master/Homebrew.sh](http://link.zhihu.com/?target=https%3A//gitee.com/cunkai/HomebrewCN/raw/master/Homebrew.sh))"
   ```

   以上command来源于知乎，输入terminal后可选择镜像进行下载

   homebrew下载完成后，只需在terminal中输入command： `brew install opencv`即可快速下载并安装OpenCV环境

   OpenCV官方安装教程（不使用homebrew，但过程较为繁琐且极易出错，不推荐）：https://docs.opencv.org/master/d0/db2/tutorial_macos_install.html

   homebrew官方安装指令（不在中国或有非常稳定的VPN可以使用）：

   ```shell
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. 在pycharm中添加interpreter：

   - opencv-python
   - mediapipe

   注：mediapipe和OpenCV需使用python3环境，所以需要在电脑上先安装好python3，并在pycharm的环境中选择python3作为interpreter

   ![preferences](https://github.com/SLiu2000/ROS-robot-movement-control-by-hand-tracking/blob/main/images/preferences.png)

