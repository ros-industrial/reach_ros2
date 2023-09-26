cmake_minimum_required(VERSION 3.7)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

option(BUILD_PYTHON "Build Python bindings" ON)

# Python dependencies need to be found first
if(BUILD_PYTHON)
  find_package(Python REQUIRED COMPONENTS Interpreter Development)
  find_package(PythonLibs 3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS python numpy)
  find_package(ament_cmake_python REQUIRED)
endif()

find_package(reach REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ROS2_DEPS
    tf2_eigen
    interactive_markers
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    rclcpp
    sensor_msgs
    visualization_msgs)

find_package(ament_cmake REQUIRED)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# Create a variable list for storing exportable library target names Note: this list should not include plugin
# libraries, which are not for linking
set(TARGETS "")

# Plugin Implementations
add_library(
  ${PROJECT_NAME} SHARED
  src/utils.cpp
  # Evaluator
  src/evaluation/manipulability_moveit.cpp
  src/evaluation/joint_penalty_moveit.cpp
  src/evaluation/distance_penalty_moveit.cpp
  # IK Solver
  src/ik/moveit_ik_solver.cpp
  # Display
  src/display/ros_display.cpp
  # Target pose generator
  src/target_pose_generator/transformed_point_cloud_target_pose_generator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC reach::reach)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS2_DEPS})
list(APPEND TARGETS ${PROJECT_NAME})

# Plugin Library
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME})

# Reach study node
add_executable(${PROJECT_NAME}_node src/reach_study_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC reach::reach yaml-cpp ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC ${ROS2_DEPS})

if(BUILD_PYTHON)
  message("Building Python bindings")

  # Call ament_python_install_package first to set PYTHON_INSTALL_DIR
  ament_python_install_package(${PROJECT_NAME})
  add_subdirectory(src/python)

  # Install Python executables
  install(PROGRAMS scripts/reach_ros_node.py scripts/demo.py DESTINATION lib/${PROJECT_NAME})
endif()

# Install the libraries
install(TARGETS ${TARGETS} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_plugins DESTINATION lib)

# Install the executable(s)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# Install the headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install the support directories
install(DIRECTORY launch demo DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS2_DEPS})
ament_package()
