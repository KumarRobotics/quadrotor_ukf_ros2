cmake_minimum_required(VERSION 3.8)
project(quadrotor_ukf_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
add_executable(quadrotor_ukf_ros2 src/quadrotor_ukf_node.cpp src/quadrotor_ukf.cpp src/vio_utils.cpp)

ament_target_dependencies(quadrotor_ukf_ros2 
  rclcpp 
  geometry_msgs 
  nav_msgs std_msgs 
  sensor_msgs 
  tf2_ros
)

target_link_libraries(quadrotor_ukf_ros2 Eigen3::Eigen)

install(TARGETS
  quadrotor_ukf_ros2
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

