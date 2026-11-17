cmake_minimum_required(VERSION 3.16)
project(prm_multigoal_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(parallel_multigoal_planner_msgs REQUIRED)
find_package(ik_solver_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(ompl REQUIRED)
# OMPL >= 1.7 exports ompl::ompl; OMPL 1.6 (Humble) only sets OMPL_INCLUDE_DIRS / OMPL_LIBRARIES.
if(NOT TARGET ompl::ompl)
  add_library(ompl::ompl INTERFACE IMPORTED)
  set_target_properties(ompl::ompl PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OMPL_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${OMPL_LIBRARIES}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/goal_set_builder.cpp
  src/${PROJECT_NAME}/multigoal_lazy_prm_star.cpp
  src/${PROJECT_NAME}/moveit_state_validity_checker.cpp
  src/${PROJECT_NAME}/scene_hash.cpp
  src/${PROJECT_NAME}/prm_multigoal_planner_action_server.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ompl::ompl
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${moveit_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${parallel_multigoal_planner_msgs_TARGETS}
  ${ik_solver_msgs_TARGETS}
  ${moveit_core_TARGETS}
  ${moveit_ros_planning_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  moveit_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  parallel_multigoal_planner_msgs
  ik_solver_msgs
  moveit_core
  moveit_ros_planning
  ompl
)

ament_package()
