cmake_minimum_required(VERSION 2.8.3)
project(gazebo_radiation_plugins)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_dev
  gazebo_ros
  pluginlib
  message_generation
  
)

find_package(Boost REQUIRED COMPONENTS filesystem)

add_message_files(
  FILES
  Simulated_Radiation_Msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} radiation_source radiation_source_plugin radiation_sensor radiation_sensor_plugin
  CATKIN_DEPENDS gazebo_dev pluginlib rosbash message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(custom_sensor_preloader SHARED src/CustomSensorPreloader.cpp)
add_dependencies(custom_sensor_preloader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_sensor_preloader ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(radiation_source  SHARED src/RadiationSource.cpp)
add_dependencies(radiation_source ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radiation_source ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(radiation_source_plugin SHARED src/RadiationSourcePlugin.cpp)
add_dependencies(radiation_source_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radiation_source_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(radiation_sensor SHARED src/RadiationSensor.cpp)
add_dependencies(radiation_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radiation_sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(radiation_sensor_plugin SHARED src/RadiationSensorPlugin.cpp)
add_dependencies(radiation_sensor_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radiation_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})


file(GENERATE
  OUTPUT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/gzserver"
  INPUT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gzserver.in")

file(GENERATE
  OUTPUT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/gazebo"
  INPUT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gazebo.in")

install(TARGETS custom_sensor_preloader radiation_source radiation_source_plugin radiation_sensor radiation_sensor_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/gzserver"
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/gazebo"
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES radiation_source.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES radiation_sensor.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
