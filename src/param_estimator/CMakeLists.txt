cmake_minimum_required(VERSION 3.5)
project(online_param_estimator)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Params.msg"
        DEPENDENCIES std_msgs geometry_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/estimateInertia>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/estimateInertia/interface>
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>)

file(GLOB LIB_SRCS lib/estimateInertia/*.cpp)

add_executable(param_estimator
        lib/EKF_estimator/EKF_estimator.cpp src/estimator_node.cpp ${LIB_SRCS})

ament_target_dependencies(
        param_estimator
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "px4_msgs"
)

#add_executable(estimator_test
#        lib/EKF_estimator/EKF_estimator.cpp src/test.cpp ${LIB_SRCS})
#ament_target_dependencies(
#        estimator_test
#        "rclcpp"
#)
#
add_executable(estimator_test_ideal
        lib/EKF_estimator/EKF_estimator.cpp src/test_ideal_sim.cpp ${LIB_SRCS})
ament_target_dependencies(
        estimator_test_ideal
        "rclcpp"
)

rosidl_target_interfaces(param_estimator
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS param_estimator estimator_test_ideal
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY cfg
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
