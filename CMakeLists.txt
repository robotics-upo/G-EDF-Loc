cmake_minimum_required(VERSION 3.8)
project(g_edf_loc)

# Estandarización C++17 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Flags seguras. Eliminado -march=native para evitar conflictos binarios.
  add_compile_options(-Wall -Wextra -Wpedantic -O3) 
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters search kdtree registration)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

# VDT Library Search
find_path(VDT_INCLUDE_DIRS NAMES vdt/exp.h PATHS /usr/local/include)
find_library(VDT_LIBRARIES NAMES vdt PATHS /usr/local/lib)

if(NOT VDT_INCLUDE_DIRS OR NOT VDT_LIBRARIES)
  message(FATAL_ERROR "VDT library not found in /usr/local")
else()
  message(STATUS "VDT Found: ${VDT_LIBRARIES}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
  
include_directories(include)

# ==============================================================================
# 1. Base LiDAR Localization Component
# ==============================================================================
add_library(lidar_localization_component SHARED
  src/LiDAR_localization.cpp
  src/utils.cpp
  src/scan_processing.cpp
)
target_include_directories(lidar_localization_component PRIVATE include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${VDT_INCLUDE_DIR})
target_link_libraries(lidar_localization_component ${PCL_LIBRARIES} ${CERES_LIBRARIES} glog OpenMP::OpenMP_CXX ${VDT_LIB})
ament_target_dependencies(lidar_localization_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  pcl_conversions
  pcl_ros
)
rclcpp_components_register_node(lidar_localization_component PLUGIN "g_edf_loc::LiDARLocalization" EXECUTABLE lidar_localization_node)

# ==============================================================================
# 2. Map Publisher Node
# ==============================================================================
add_executable(map_publisher_node src/map_publisher_node.cpp)
target_include_directories(map_publisher_node PRIVATE include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${VDT_INCLUDE_DIR})
target_link_libraries(map_publisher_node ${PCL_LIBRARIES} glog OpenMP::OpenMP_CXX ${VDT_LIB})
ament_target_dependencies(map_publisher_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  pcl_conversions
)

# ==============================================================================
# 4. No IMU Variant
# ==============================================================================
add_library(lidar_localization_no_imu_component SHARED
  src/LiDAR_localization_no_imu.cpp
  src/utils.cpp
  src/scan_processing.cpp
)
target_include_directories(lidar_localization_no_imu_component PRIVATE include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${VDT_INCLUDE_DIR})
target_link_libraries(lidar_localization_no_imu_component ${PCL_LIBRARIES} ${CERES_LIBRARIES} glog OpenMP::OpenMP_CXX ${VDT_LIB})
ament_target_dependencies(lidar_localization_no_imu_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  pcl_conversions
  pcl_ros
)
rclcpp_components_register_node(lidar_localization_no_imu_component PLUGIN "g_edf_loc::LiDARLocalizationNoIMU" EXECUTABLE lidar_localization_no_imu_node)

# ==============================================================================
# Install Rules
# ==============================================================================
install(TARGETS 
  lidar_localization_component 
  lidar_localization_no_imu_component  
  map_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()