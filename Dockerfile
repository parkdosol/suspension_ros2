# 베이스 이미지로 ros:foxy 사용
FROM ros:foxy

# 기본 패키지 업데이트 및 필수 툴 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    curl \
    wget \
    vim \
    tree \
    x11-apps \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Bash 프롬프트를 설정 (노란색 텍스트, 파란색 파일 경로)
RUN echo 'export PS1="\[\033[1;33m\]\u@\h:\[\033[1;34m\]\w\[\033[0m\]\$ "' >> ~/.bashrc

# 작업 디렉토리 생성 및 설정
WORKDIR /root/suspension_ros2

# Python 의존성 설치
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# 환경 변수 설정
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=foxy \
    DISPLAY=${DISPLAY} \
    QT_X11_NO_MITSHM=1

# ROS 2 작업 공간 생성 및 설정
RUN mkdir -p /root/suspension_ros2/src

# 작업 디렉토리 설정
WORKDIR /root/suspension_ros2

# 엔트리포인트 설정
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# 기본 실행 명령어
CMD ["bash"]

