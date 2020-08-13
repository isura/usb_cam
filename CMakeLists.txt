cmake_minimum_required(VERSION 3.5)
project(usb_cam)

## find dependencies
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avutil  libavutil REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###########
## Build ##
###########

# libraries
SET( Libraries
        ament_cmake
        image_transport
        rclcpp
        std_msgs
        std_srvs
        sensor_msgs
        camera_info_manager )

include_directories(include
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
ament_target_dependencies(${PROJECT_NAME} ${Libraries})
target_link_libraries(${PROJECT_NAME}
  ${avutil_LIBRARIES}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avutil_LIBRARIES}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()