cmake_minimum_required(VERSION 2.8.3)
project(morobot_service)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  std_srvs
  message_generation
  message_runtime
  tf
  nav_msgs
  geometry_msgs
  move_base
  visualization_msgs 
  dynamic_reconfigure
)

add_service_files(
  FILES
  setlocation.srv
  getlocation.srv
  gotocancel.srv
  gotolocation.srv
  patrol.srv
  setlocation.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

catkin_package(
  LIBRARIES morobot_service
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_generation message_runtime
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(morobot_service src/morobot_service.cpp)
add_dependencies(morobot_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(morobot_service ${catkin_LIBRARIES})


