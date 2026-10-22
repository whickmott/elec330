cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/encoder_wheels.py DESTINATION lib/${PROJECT_NAME})

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(featherwing_motors SHARED src/featherwing_motors.cpp)
target_compile_features(featherwing_motors PUBLIC cxx_std_17)
ament_target_dependencies(featherwing_motors
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/featherwing_plugin.xml"
  "<library path=\"featherwing_motors\"><class name=\"my_robot_bringup/FeatherWingMotors\" type=\"my_robot_bringup::FeatherWingMotors\" base_class_type=\"hardware_interface::SystemInterface\"/></library>")

install(TARGETS featherwing_motors LIBRARY DESTINATION lib)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/featherwing_plugin.xml"
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/teleop.py DESTINATION lib/${PROJECT_NAME})
ament_index_register_resource(hardware_interface__pluginlib__plugin
  CONTENT "share/${PROJECT_NAME}/featherwing_plugin.xml\n")
  
ament_package()