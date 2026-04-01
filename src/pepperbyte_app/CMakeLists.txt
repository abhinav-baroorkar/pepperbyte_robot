# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
cmake_minimum_required(VERSION 3.8)
project(pepperbyte_app)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch webapp
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
