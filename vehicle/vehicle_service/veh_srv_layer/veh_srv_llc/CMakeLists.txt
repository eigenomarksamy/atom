cmake_minimum_required(VERSION 3.0.2)
project(veh_srv_llc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  veh_srv_ros_types
)

catkin_package()

set(INC_DIR
    src)

include_directories(
  ${INC_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(SRC
    src/veh_srv_llc.cpp)


add_executable(veh_srv_llc ${SRC})

target_link_libraries(veh_srv_llc ${catkin_LIBRARIES})

install(TARGETS veh_srv_llc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})