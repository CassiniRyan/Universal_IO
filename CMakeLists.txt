cmake_minimum_required(VERSION 3.16)
project(g1_io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

option(USE_SDK2 "Link unitree_sdk2 for BMS/board/pressure DDS topics" OFF)

# ── ROS2 ──────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_go REQUIRED)

set(ROS_DEPS rclcpp std_msgs sensor_msgs geometry_msgs
             tf2_ros tf2_geometry_msgs unitree_hg unitree_go)

# ── Include paths ──────────────────────────────────────────────────────────
include_directories(
  core/bridge/cpp_middleware/include
  core/bridge/ros2_msg_types
  core/utils/cpp/include
)

# ── Optional unitree_sdk2 ──────────────────────────────────────────────────
if(USE_SDK2)
  set(SDK2_DIR "${CMAKE_SOURCE_DIR}/third_party/unitree_sdk2")
  if(EXISTS "${SDK2_DIR}/include")
    include_directories(
      ${SDK2_DIR}/include
      ${SDK2_DIR}/thirdparty/include
    )
    # Detect architecture for pre-built lib
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
      set(SDK2_LIB_DIR "${SDK2_DIR}/lib/aarch64")
      set(DDS_LIB_DIR  "${SDK2_DIR}/thirdparty/lib/aarch64")
    else()
      set(SDK2_LIB_DIR "${SDK2_DIR}/lib/x86_64")
      set(DDS_LIB_DIR  "${SDK2_DIR}/thirdparty/lib/x86_64")
    endif()
    add_definitions(-DG1_IO_HAS_SDK2)
    set(SDK2_LIBS
      "${SDK2_LIB_DIR}/libunitree_sdk2.a"
      "${DDS_LIB_DIR}/libddsc.so"
      "${DDS_LIB_DIR}/libddscxx.so"
      rt pthread)
    message(STATUS "unitree_sdk2 found — BMS/board/pressure topics enabled.")
  else()
    message(WARNING "USE_SDK2=ON but third_party/unitree_sdk2 not cloned. "
                    "BMS/board/pressure topics will be disabled.")
    set(SDK2_LIBS "")
  endif()
else()
  set(SDK2_LIBS "")
  message(STATUS "USE_SDK2=OFF — BMS/board/pressure topics disabled. "
                 "Rebuild with -DUSE_SDK2=ON after cloning unitree_sdk2.")
endif()

# ── Safety filters (separate library) ─────────────────────────────────────
add_library(safety_filters SHARED
  core/utils/cpp/src/safety_filter_0.cpp
  core/utils/cpp/src/safety_filter_1.cpp
)
ament_target_dependencies(safety_filters ${ROS_DEPS})

# ── cpp_middleware ─────────────────────────────────────────────────────────
add_executable(cpp_middleware
  core/bridge/cpp_middleware/src/G1_middleware.cpp
)
target_link_libraries(cpp_middleware safety_filters ${SDK2_LIBS})
ament_target_dependencies(cpp_middleware ${ROS_DEPS})

# ── realsense_node (optional) ──────────────────────────────────────────────
find_package(realsense2 QUIET)
if(realsense2_FOUND)
  add_executable(realsense_node core/adapters/realsense/src/realsense_node.cpp)
  target_link_libraries(realsense_node ${realsense2_LIBRARY})
  ament_target_dependencies(realsense_node ${ROS_DEPS})
  install(TARGETS realsense_node DESTINATION lib/${PROJECT_NAME})
  message(STATUS "RealSense found — realsense_node will be built.")
else()
  message(STATUS "RealSense NOT found — realsense_node skipped.")
endif()

# ── Python adapters ────────────────────────────────────────────────────────
ament_python_install_package(core)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS cpp_middleware safety_filters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  core/bridge/cpp_middleware/include/
  core/bridge/ros2_msg_types/
  core/utils/cpp/include/
  DESTINATION include
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
