cmake_minimum_required(VERSION 3.8)
project(grp_astro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Copy rviz2 config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Copy images
install(DIRECTORY
  src/img
  DESTINATION lib/${PROJECT_NAME}
)
# Copy HSV tuning
install(DIRECTORY
  src/config
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts
install( PROGRAMS 
  src/cam_driver.py
  src/cam_vision
  src/reactive_move
  src/scan2point_cloud
  src/bottle_mapping
  src/auto_discovery

  DESTINATION lib/${PROJECT_NAME}
)