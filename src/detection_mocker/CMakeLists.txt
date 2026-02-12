cmake_minimum_required(VERSION 3.8)
project(detection_mocker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Find TinyXML2 (system library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

# Create library for DetectionMocker class
add_library(detection_mocker_lib SHARED
  src/detection_mocker.cpp
  src/xml_parser.cpp
  src/transforms.cpp
  src/frustum_culler.cpp)

target_include_directories(detection_mocker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/3rdparty>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${TINYXML2_INCLUDE_DIRS})

ament_target_dependencies(detection_mocker_lib
  rclcpp
  interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
  vision_msgs
)

target_link_libraries(detection_mocker_lib Eigen3::Eigen ${TINYXML2_LIBRARIES})

# Main executable
add_executable(detection_mocker src/main.cpp)
target_link_libraries(detection_mocker detection_mocker_lib)
ament_target_dependencies(detection_mocker rclcpp)

# Static map publisher (oneshot mode)
add_executable(static_map_publisher src/static_map_publisher.cpp)
target_link_libraries(static_map_publisher detection_mocker_lib)
ament_target_dependencies(static_map_publisher rclcpp interfaces vision_msgs)

install(TARGETS detection_mocker static_map_publisher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS detection_mocker_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

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
