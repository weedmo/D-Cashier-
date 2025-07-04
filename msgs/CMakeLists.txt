cmake_minimum_required(VERSION 3.8)
project(msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# ── Interface Install ──────────────────────────
install(
  DIRECTORY srv
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# ── Message Generation ─────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AdultEvent.srv"
  "srv/ObjectInformation.srv"
  "action/PickAndPlace.action"
  DEPENDENCIES std_msgs action_msgs
)

# ── Linting (Optional) ─────────────────────────
if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
