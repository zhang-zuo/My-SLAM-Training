cmake_minimum_required(VERSION 3.0.2)
project(local_map)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf
  geometry_msgs
  nav_msgs
  sensor_msgs
)

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
#  LIBRARIES local_map
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)


include_directories(
# include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../../devel/include
    ${PROJECT_SOURCE_DIR}/../Thirdparty/
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
MESSAGE("project: " ${PROJECT_SOURCE_DIR})

add_executable(local_map
    src/local_map_node.cpp
    src/LocalMap.cpp
    src/KeyFrame.cpp
    src/MapPoint.cpp
    src/Optimizer.cpp
    src/ShowRVIZ.cpp
)

target_link_libraries(local_map
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    g2o_core g2o_stuff
    g2o_types_sba
    Sophus::Sophus
    ${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/lib/libDBoW2.so
)