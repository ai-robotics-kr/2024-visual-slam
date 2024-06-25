cmake_minimum_required(VERSION 3.8)
set(PACKAGE_NAME vio)
set(PACKAGE_VERSION 1.0.0)
set(CMAKE_BUILD_TYPE Release)

project(${PACKAGE_NAME} 
    VERSION ${PACKAGE_VERSION} 
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIB_DIR})

# Eigen3
include_directories("/usr/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIRS})

# fmt
find_package(fmt REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)

# libslam.so 파일이 있는 디렉토리 경로를 설정합니다.
set(LIBSLAMSO /2024-visual-slam/lib/)

# libslam.so 파일을 찾습니다.
find_library(SLAM_LIBRARY
    NAMES libslam.so
    PATHS ${LIBSLAMSO}
)

# libslam.so 파일을 찾았는지 확인합니다.
if(NOT SLAM_LIBRARY)
    message(FATAL_ERROR "libslam.so not found")
endif()

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES} GL GLU GLEW glut
    g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
    ${GTEST_BOTH_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    pthread
    ${CSPARSE_LIBRARY}
    fmt::fmt 
    ${SLAM_LIBRARY}
)

include_directories(${PROJECT_SOURCE_DIR}/../../../include)
include_directories(include)

# Set RPATH
set(CMAKE_INSTALL_RPATH "${LIBSLAMSO}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(vio_ros2 src/run_ros2.cpp src/rgbd_vio_node.cpp)
ament_target_dependencies(vio_ros2 rclcpp sensor_msgs cv_bridge message_filters)
target_link_libraries(vio_ros2 ${THIRD_PARTY_LIBS})

install(TARGETS
    vio_ros2
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
