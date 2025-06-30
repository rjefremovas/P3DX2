cmake_minimum_required(VERSION 3.8)
project(tp1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(navigation src/main.cpp src/Action.cpp src/Perception.cpp src/Utils.cpp)

target_include_directories(navigation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(navigation PUBLIC c_std_99 cxx_std_17)

# Link against ROS 2 and LibTorch
ament_target_dependencies(navigation
  nav_msgs
  tf2
  tf2_geometry_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(navigation 
  ${TORCH_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)
target_compile_definitions(navigation PRIVATE -D_GLIBCXX_USE_CXX11_ABI=1)

# Install the binary
install(
  TARGETS navigation
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
