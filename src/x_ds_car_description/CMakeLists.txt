cmake_minimum_required(VERSION 3.5)
project(ds_car_description)

# 기본 C 및 C++ 표준 설정
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# URDF/Xacro 설치
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


