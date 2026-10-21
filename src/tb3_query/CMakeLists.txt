cmake_minimum_required(VERSION 3.8)
project(tb3_query)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Generate custom messages ──────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticQueryResult.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ── Resolve Python install dir ────────────────────────────────────────────
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

# ── Install Python modules into the rosidl-generated package dir ──────────
# rosidl already creates tb3_query/__init__.py in the site-packages tree;
# we add query_core.py alongside it so that
# `from tb3_query.query_core import ...` works at runtime.
install(FILES
  tb3_query/query_core.py
  DESTINATION "${PYTHON_INSTALL_DIR}/tb3_query"
)

# ── Install Python executables ────────────────────────────────────────────
install(PROGRAMS
  tb3_query/semantic_query_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config ────────────────────────────────────────────────────────
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
