cmake_minimum_required(VERSION 3.8)
project(detection_mocker)

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

add_executable(detection_mocker
  src/detection_mocker.cpp
  src/xml_parser.cpp
  src/transforms.cpp
  src/frustum_culler.cpp
  include/3rdparty/tinyxml2.cpp)

target_include_directories(detection_mocker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/3rdparty>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(detection_mocker
  rclcpp
  interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
  vision_msgs
)

target_link_libraries(detection_mocker Eigen3::Eigen)

target_compile_features(detection_mocker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS detection_mocker
  DESTINATION lib/${PROJECT_NAME})

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
