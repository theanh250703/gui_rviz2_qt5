cmake_minimum_required(VERSION 3.8)
project(gui_agv)

# Yêu cầu chuẩn C++
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Kích hoạt các tính năng tự động của Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# Cảnh báo biên dịch
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm các gói phụ thuộc
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(nav2_map_server REQUIRED)


# Tìm các tệp nguồn, tiêu đề, UI và tài nguyên
file(GLOB PRO_SOURCE_DIR src/*.cpp)
file(GLOB PRO_HEADER_DIR include/*.h include/*.hpp)
file(GLOB PRO_UI_DIR ui/*.ui)
file(GLOB PRO_RESOURCE_DIR resource/*.qrc)

# Thêm thư mục include
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Xử lý các tệp UI, MOC và tài nguyên Qt
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_HEADER_DIR})
qt5_add_resources(QT_RESOURCE_HPP ${PRO_RESOURCE_DIR})

# Tạo mục tiêu thực thi
add_executable(${PROJECT_NAME}
  ${PRO_SOURCE_DIR}
  ${PRO_HEADER_DIR}
  ${QT_UI_HPP}
  ${QT_MOC_HPP}
  ${QT_RESOURCE_HPP}
)

# Liên kết với Qt5 và ROS
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  nav2_map_server
)


install(DIRECTORY
    # config
    # launch
    DESTINATION share/${PROJECT_NAME})

# Kiểm tra build
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Đóng gói ament
ament_package()