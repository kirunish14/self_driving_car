cmake_minimum_required(VERSION 3.8)

project(self_driving_car)

# =========================================================
# C / C++
# =========================================================

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# DEPENDENCIES
# =========================================================

find_package(ament_cmake REQUIRED)

# =========================================================
# INSTALL PACKAGE DIRECTORIES
# =========================================================

install(
  DIRECTORY
    description
    launch
    config
    worlds
    maps
    behavior_trees
    firmware
    docs
    web
  DESTINATION
    share/${PROJECT_NAME}
)

# =========================================================
# PYTHON EXECUTABLES
# =========================================================

install(
  PROGRAMS
    tf_relay.py
    twist_relay.py
    goal_tracker.py
    car_bridge.py
    dashboard_node.py
    self_driving_car/lane_detector.py
    self_driving_car/lane_follower.py
    self_driving_car/indoor_lane.py
    self_driving_car/indoor_lane_node.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# =========================================================
# TESTING
# =========================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

endif()

# =========================================================
# PACKAGE
# =========================================================

ament_package()
