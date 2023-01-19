cmake_minimum_required(VERSION 3.0.2)
project(feature_tracking)

#cmake_policy(SET CMP0074 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  message_generation
)

# 下面是添加的自定义数据类型
add_service_files(
    FILES
    Track_LocalMap.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(OpenCV)
MESSAGE("OpenCV dir: " ${OpenCV_INCLUDE_DIR})
find_package(Sophus REQUIRED)

include_directories("/usr/include/eigen3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES feature_tracking
#  CATKIN_DEPENDS cv_bridge roscpp std_msgs message_runtime
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../devel/include
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
MESSAGE("project: " ${PROJECT_SOURCE_DIR})


add_executable(feature_tracking
src/feature_tracking_node.cpp
src/Frame.cpp
src/MapPoint.cpp
src/Tracking.cpp
src/Optimizer.cpp
)
target_link_libraries(feature_tracking
${catkin_LIBRARIES}
${OpenCV_LIBS}
#${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so
g2o_core g2o_stuff
g2o_types_sba
Sophus::Sophus
)
#add_dependencies(feature_tracking ${PROJECT_NAME}_generate_messages_cpp)
