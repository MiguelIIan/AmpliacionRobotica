    #   Author: Juan M. Gandarias (http://jmgandarias.com)
    #   email: jmgandarias@uma.es

    cmake_minimum_required(VERSION 3.8)
    project(uma_arm_control)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(Eigen3 REQUIRED)

    include_directories(${EIGEN3_INCLUDE_DIR})

    ## COMPILE
    add_executable(uma_arm_dynamics src/uma_arm_dynamics.cpp)
    add_executable(gravity_compensation src/gravity_compensation.cpp)
    add_executable(dynamics_cancellation src/dynamics_cancellation.cpp)

    ament_target_dependencies(uma_arm_dynamics 
      rclcpp
      std_msgs
      sensor_msgs
      geometry_msgs
      Eigen3
    )

    ament_target_dependencies(gravity_compensation 
      rclcpp
      std_msgs
      sensor_msgs
      Eigen3
    )

    ament_target_dependencies(dynamics_cancellation 
      rclcpp
      std_msgs
      sensor_msgs
      geometry_msgs
      Eigen3
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

    install(TARGETS uma_arm_dynamics dynamics_cancellation gravity_compensation
      DESTINATION lib/${PROJECT_NAME}
      )

    install(DIRECTORY 
      launch
      config
      DESTINATION share/${PROJECT_NAME}/
    )

    ament_package()
