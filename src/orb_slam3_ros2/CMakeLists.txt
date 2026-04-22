cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

# ==============================
#  Compiler and build settings
# ==============================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
#  ROS 2 and external dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# Resolve OpenCV for headers. We intentionally avoid linking OpenCV directly
# in this package to prevent mixing OpenCV ABIs with prebuilt ORB_SLAM3/cv_bridge.
set(CV_BRIDGE_OPENCV_VERSION "")
if(DEFINED OpenCV_VERSION)
  set(CV_BRIDGE_OPENCV_VERSION "${OpenCV_VERSION}")
endif()

find_package(OpenCV 4 REQUIRED)
message(STATUS "Resolved OpenCV version: ${OpenCV_VERSION}")
if(CV_BRIDGE_OPENCV_VERSION AND NOT OpenCV_VERSION VERSION_EQUAL CV_BRIDGE_OPENCV_VERSION)
  message(WARNING
    "OpenCV mismatch detected (cv_bridge: ${CV_BRIDGE_OPENCV_VERSION}, this package: ${OpenCV_VERSION}). "
    "Direct OpenCV linkage is disabled in orb_slam3_ros2 to reduce ABI conflicts."
  )
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# ORB_SLAM3 - resolve the checkout from common workspace locations or a user override
set(ORB_SLAM3_ROOT_DIR "" CACHE PATH "Path to the ORB_SLAM3 checkout")
if(NOT ORB_SLAM3_ROOT_DIR)
  set(_orb_slam3_candidates
    ${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/ORB_SLAM3
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cv/third_party/ORB_SLAM3
  )

  foreach(_candidate IN LISTS _orb_slam3_candidates)
    if(EXISTS "${_candidate}/include/System.h")
      set(ORB_SLAM3_ROOT_DIR "${_candidate}")
      break()
    endif()
  endforeach()
endif()

if(NOT ORB_SLAM3_ROOT_DIR OR NOT EXISTS "${ORB_SLAM3_ROOT_DIR}/include/System.h")
  message(FATAL_ERROR
    "Could not locate ORB_SLAM3. Set ORB_SLAM3_ROOT_DIR to a checkout containing include/System.h."
  )
endif()

message(STATUS "Using ORB_SLAM3 from: ${ORB_SLAM3_ROOT_DIR}")
set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/include
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/include
)
set(ORB_SLAM3_LIBRARY ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so)
set(DBOW2_LIBRARY ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
set(G2O_LIBRARY ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so)

# ==============================
#  Dependency list for ament
# ==============================
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  message_filters
)

# ==============================
#  Build executable node
# ==============================
add_executable(orb_slam_node src/orb_slam_node.cpp)

ament_target_dependencies(orb_slam_node ${dependencies})

# Link with ORB_SLAM3 and external libs
target_link_libraries(orb_slam_node
  ${ORB_SLAM3_LIBRARY}            
  ${DBOW2_LIBRARY}               
  ${G2O_LIBRARY}                  
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  cv_bridge::cv_bridge           
  -lboost_serialization
  -lcrypto
)

# ==============================
#  Include directories (using direct paths)
# ==============================
target_include_directories(orb_slam_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

install(TARGETS
  orb_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ==============================
#  Package export for ament
# ==============================
ament_package()
