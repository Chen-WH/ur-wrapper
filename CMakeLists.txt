cmake_minimum_required(VERSION 3.10)
project(ur-wrapper)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install targets
install(PROGRAMS
  script/home.py
  script/joint_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
