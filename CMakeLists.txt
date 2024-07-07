cmake_minimum_required(VERSION 3.8)
project(cam_port_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
# ROS Dependencies =================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)


# Non ROS Dependencies =============================================

# Spinnaker deps
set(SPINNAKER_INCLUDE_DIR "/opt/spinnaker/include" CACHE PATH "Spinnaker Include Directory")
set(SPINNAKER_LIB_DIR "/opt/spinnaker/lib/" CACHE PATH "Spinnaker Libs Directory")
file(GLOB SPINNAKER_LIB_FILES /opt/spinnaker/lib/*.so)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  find_package(LibUnwind REQUIRED)
endif()

find_package(OpenCV REQUIRED)
find_package ( Boost COMPONENTS filesystem system serialization log REQUIRED )

# ===============================================


include_directories(include)
include_directories(${SPINNAKER_INCLUDE_DIR})

file(GLOB SOURCES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

link_directories(${SPINNAKER_LIB_DIR})

install(
  DIRECTORY include
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME} ${SPINNAKER_LIB_FILES})

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  ament_target_dependencies(${PROJECT_NAME} rclcpp Boost image_transport OpenCV LibUnwind cv_bridge)
else()
  ament_target_dependencies(${PROJECT_NAME} rclcpp Boost image_transport OpenCV cv_bridge)
endif()

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
