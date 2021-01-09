# docker build -t ros2:foxy.

# Use athrill-device-v850 as base image
FROM kanetugu2015/athrill-device-v850:v1.0.0 as athrill-device-v850-builder

ENV ROS_DISTRO foxy

# ロケールのセットアップ
RUN apt-get update && apt-get install -y locales && \
    dpkg-reconfigure locales && \
    locale-gen ja_JP ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8
ENV LC_ALL   ja_JP.UTF-8
ENV LANG     ja_JP.UTF-8
ENV LANGUAGE ja_JP.UTF-8

# APTソースリストの設定
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl http://repo.ros2.org/repos.key | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu \
    `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update

# ROS2パッケージのインストール
RUN export ROS_DISTRO=foxy && \
    apt-get install -y ros-$ROS_DISTRO-desktop \
    python3-colcon-common-extensions python3-rosdep python3-argcomplete && \
    rosdep init && \
    rosdep update

## 環境設定
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
WORKDIR /root/workspace
