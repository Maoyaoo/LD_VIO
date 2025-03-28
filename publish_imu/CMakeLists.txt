cmake_minimum_required(VERSION 3.0.2)
project(publish_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2
  sensor_msgs  # 需要添加这个依赖
)

# 添加 serial 库的查找
find_package(serial REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf2 sensor_msgs
)

# 设置可执行文件输出路径
set(EXECUTABLE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})

add_executable(publish_imu_jy901s_node src/publish_imu_jy901s_node.cpp)
add_executable(publish_imu_jy901b_node src/publish_imu_jy901b_node.cpp)
add_executable(publish_imu_jy901b_node2 src/publish_imu_jy901b_node2.cpp)
add_executable(publish_imu_jy901b_node3 src/publish_imu_jy901b_node3.cpp)

target_link_libraries(publish_imu_jy901s_node
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

target_link_libraries(publish_imu_jy901b_node
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

target_link_libraries(publish_imu_jy901b_node2
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

target_link_libraries(publish_imu_jy901b_node3
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

