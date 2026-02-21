cmake_minimum_required(VERSION 3.8)
project(ai_msc_robotics_assignments_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_program(PYTHON python3)

ament_python_install_package(${PROJECT_NAME})
ament_package()

install(DIRECTORY scripts/
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING
      PATTERN "*.py"
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
