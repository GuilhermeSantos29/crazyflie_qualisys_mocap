cmake_minimum_required(VERSION 3.8)
project(qualisys_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-unused-parameter
    -Wno-deprecated-declarations
    -Wno-format-truncation
    -Wno-restrict
    -Wno-missing-field-initializers
    -Wno-class-memaccess
    -Wno-implicit-fallthrough
  )
endif()
set(CMAKE_CXX_STANDARD 17)

# Dependências ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Dependências externas
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

# --- Gera as mensagens PRIMEIRO ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Subject.msg"
  "msg/Marker.msg"
  "msg/Markers.msg"
  "srv/SetPose.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# --- Biblioteca do protocolo QTM (código legado — não muda) ---
add_library(qualisys_protocol
  include/qualisys_ros2/RTProtocol.cpp
  include/qualisys_ros2/NBC_Markup.cpp
  include/qualisys_ros2/RTPacket.cpp
  include/qualisys_ros2/Network.cpp
)

target_include_directories(qualisys_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qualisys_ros2>
  $<INSTALL_INTERFACE:include>
)

# --- Typesupport para mensagens geradas ---
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- Driver principal ---
add_library(qualisys_driver
  src/QualisysDriver.cpp
)

target_include_directories(qualisys_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qualisys_ros2>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(qualisys_driver
  rclcpp geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
)

target_link_libraries(qualisys_driver
  qualisys_protocol
  "${cpp_typesupport_target}"
)

add_dependencies(qualisys_driver ${PROJECT_NAME}__rosidl_typesupport_cpp)

# --- Odometria ---
add_library(qualisys_odom_lib
  src/QualisysOdom.cpp
  src/KalmanFilter.cpp
  
)

target_include_directories(qualisys_odom_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qualisys_ros2>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(qualisys_odom_lib
  rclcpp geometry_msgs nav_msgs
)

target_link_libraries(qualisys_odom_lib
  "${cpp_typesupport_target}"
)

add_dependencies(qualisys_odom_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

# --- Calibração ---
add_library(qualisys_calib_lib
  src/QualisysCalib.cpp
)

target_include_directories(qualisys_calib_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qualisys_ros2>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${YAMLCPP_INCLUDE_DIRS}
)

ament_target_dependencies(qualisys_calib_lib
  rclcpp geometry_msgs std_srvs
)

target_link_libraries(qualisys_calib_lib
  "${cpp_typesupport_target}"
  ${YAMLCPP_LIBRARIES}
)

add_dependencies(qualisys_calib_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

# --- Executáveis ---
add_executable(qualisys_node src/qualisys_node.cpp)
target_link_libraries(qualisys_node qualisys_driver)
ament_target_dependencies(qualisys_node rclcpp)

add_executable(qualisys_odom_node src/qualisys_odom_node.cpp)
target_link_libraries(qualisys_odom_node qualisys_odom_lib)
ament_target_dependencies(qualisys_odom_node rclcpp)

add_executable(qualisys_calib_node src/qualisys_calib_node.cpp)
target_link_libraries(qualisys_calib_node qualisys_calib_lib)
ament_target_dependencies(qualisys_calib_node rclcpp)

# --- Instalação ---
install(TARGETS
  qualisys_protocol
  qualisys_driver
  qualisys_odom_lib
  qualisys_calib_lib
  qualisys_node
  qualisys_odom_node
  qualisys_calib_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY calib/
  DESTINATION share/${PROJECT_NAME}/calib
)

ament_package()
