cmake_minimum_required(VERSION 3.8)
project(robo_haul)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= Dependencies =================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(gazebo_ros REQUIRED)



# ================= A* C++ Extension =================
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(astar_cpp
  src/astar.cpp
)

target_compile_options(astar_cpp PRIVATE -O3 -march=native)

install(TARGETS astar_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ================= C++ Nodes =================
# مثال إذا عندك ملفات C++ في src/

add_executable(move_forward src/move_forward.cpp)

ament_target_dependencies(move_forward
  rclcpp
  geometry_msgs
)

install(TARGETS
  move_forward
  DESTINATION lib/${PROJECT_NAME}
)

# obstacle avoidance node
add_executable(engelden_kacinma src/engelden_kacinma_node.cpp)

ament_target_dependencies(engelden_kacinma
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  engelden_kacinma
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS
#   navigation_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ================= Python Nodes =================
# install(PROGRAMS
#   scripts/dashboard_nav.py
#   scripts/web_dashboard.py
#   scripts/adaptive_speed_controller.py   # ← ضيف هاد
#   DESTINATION lib/${PROJECT_NAME}
# )

# ================= Install Resources =================
install(DIRECTORY
  launch
  urdf
  worlds
  config
  rviz
  maps
  models
  DESTINATION share/${PROJECT_NAME}
)

# ================= Testing =================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()