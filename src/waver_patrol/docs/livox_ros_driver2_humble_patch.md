# Livox ROS Driver 2 Humble Build Patch

`src/livox_ros_driver2` is stored as a gitlink in this workspace, so local edits
inside that nested repository are not committed as normal FSD_Vehicle source
files.  The workspace build was validated with the following local Humble patch
applied to `src/livox_ros_driver2/CMakeLists.txt`.

Apply this patch inside `src/livox_ros_driver2` if a clean checkout fails to
build Livox on ROS 2 Humble:

```diff
diff --git a/CMakeLists.txt b/CMakeLists.txt
index 99a8ccc..6fd6dda 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -191,6 +191,13 @@ else(ROS_EDITION STREQUAL "ROS2")
   cmake_minimum_required(VERSION 3.14)
   project(livox_ros_driver2)
+
+  # ROS 2 Humble users normally run plain `colcon build` without passing the
+  # vendor build flag `-DHUMBLE_ROS=humble`.  Default it from ROS_DISTRO so the
+  # Humble rosidl branch is used in workspace builds.
+  if(NOT DEFINED HUMBLE_ROS AND "$ENV{ROS_DISTRO}" STREQUAL "humble")
+    set(HUMBLE_ROS "humble")
+  endif()
+
   # Default to C99
   if(NOT CMAKE_C_STANDARD)
     set(CMAKE_C_STANDARD 99)
@@ -286,6 +293,8 @@ else(ROS_EDITION STREQUAL "ROS2")
     rosidl_get_typesupport_target(cpp_typesupport_target
     ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
     target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
+    set(LIVOX_INTERFACE_TARGET "${cpp_typesupport_target}")
+    set(LIVOX_INTERFACES_INCLUDE_DIRECTORIES "")
   else()
     set(LIVOX_INTERFACE_TARGET "${LIVOX_INTERFACES}__rosidl_typesupport_cpp")
     add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})
@@ -333,4 +342,4 @@ else(ROS_EDITION STREQUAL "ROS2")
     launch_ROS2
   )
+
-endif()
\ No newline at end of file
+endif()
```
