cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_core_examples)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosdyn_core
  subscription_notifier
  eigen_matrix_utils
)
catkin_package(
 CATKIN_DEPENDS roscpp rosdyn_core subscription_notifier eigen_matrix_utils
 DEPENDS 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(inverse_dynamics src/inverse_dynamics.cpp)
add_dependencies(inverse_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inverse_dynamics
  ${catkin_LIBRARIES}
)


add_executable(inverse_kinematics src/ik.cpp)
add_dependencies(inverse_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inverse_kinematics
  ${catkin_LIBRARIES}
)


add_executable(external_wrench_test src/external_wrench_test.cpp)
add_dependencies(external_wrench_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(external_wrench_test
  ${catkin_LIBRARIES}
)





