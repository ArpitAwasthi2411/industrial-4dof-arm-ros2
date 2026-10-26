cmake_minimum_required(VERSION 3.8)
project(industrial_4dof_arm)

find_package(ament_cmake REQUIRED)

# Install model, launch and RViz resources into share/industrial_4dof_arm
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_model test/test_urdf_model.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
