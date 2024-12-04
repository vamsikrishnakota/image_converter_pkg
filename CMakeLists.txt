cmake_minimum_required(VERSION 3.8)
project(image_converter_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_conversion_node src/image_converter_node.cpp)

find_package(std_srvs REQUIRED)

ament_target_dependencies(image_conversion_node 
    rclcpp 
    image_transport 
    sensor_msgs 
    std_msgs 
    cv_bridge 
    OpenCV 
    std_srvs
)


install(TARGETS
  image_conversion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
