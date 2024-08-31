cmake_minimum_required(VERSION 3.8)
project(cuboid_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 実行ファイルのターゲットを追加
add_executable(ave_cov src/ave_cov.cpp)
ament_target_dependencies(ave_cov rclcpp)

# ビルドターゲットのインストール
install(TARGETS ave_cov
  DESTINATION lib/${PROJECT_NAME}
)

# Python スクリプトのインストール
install(PROGRAMS
  scripts/d_op.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch ファイルのインストール
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
