#!/bin/bash
set -e  # 스크립트 실행 중 오류가 발생하면 즉시 종료

# ROS 2 설치 경로 설정
source /opt/ros/$ROS_DISTRO/setup.bash

# 작업 공간의 환경 설정 파일 로드 (빌드된 경우)
if [ -f "/root/suspension_ros2_ws/install/setup.bash" ]; then
    source /root/suspension_ros2_ws/install/setup.bash
else
    echo "Workspace not built yet. Skipping workspace setup."
fi

# 입력된 명령 실행
exec "$@"

