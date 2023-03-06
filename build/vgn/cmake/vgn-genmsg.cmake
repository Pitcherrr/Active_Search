# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vgn: 1 messages, 3 services")

set(MSG_I_FLAGS "-Ivgn:/home/tom/dev_ws/thesis_ws/src/vgn/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vgn_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" NAME_WE)
add_custom_target(_vgn_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vgn" "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" NAME_WE)
add_custom_target(_vgn_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vgn" "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header"
)

get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" NAME_WE)
add_custom_target(_vgn_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vgn" "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header"
)

get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" NAME_WE)
add_custom_target(_vgn_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vgn" "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion:vgn/GraspConfig"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn
)

### Generating Services
_generate_srv_cpp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn
)
_generate_srv_cpp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn
)
_generate_srv_cpp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn
)

### Generating Module File
_generate_module_cpp(vgn
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vgn_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vgn_generate_messages vgn_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" NAME_WE)
add_dependencies(vgn_generate_messages_cpp _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_cpp _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_cpp _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" NAME_WE)
add_dependencies(vgn_generate_messages_cpp _vgn_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vgn_gencpp)
add_dependencies(vgn_gencpp vgn_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vgn_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn
)

### Generating Services
_generate_srv_eus(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn
)
_generate_srv_eus(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn
)
_generate_srv_eus(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn
)

### Generating Module File
_generate_module_eus(vgn
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(vgn_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(vgn_generate_messages vgn_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" NAME_WE)
add_dependencies(vgn_generate_messages_eus _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_eus _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_eus _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" NAME_WE)
add_dependencies(vgn_generate_messages_eus _vgn_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vgn_geneus)
add_dependencies(vgn_geneus vgn_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vgn_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn
)

### Generating Services
_generate_srv_lisp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn
)
_generate_srv_lisp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn
)
_generate_srv_lisp(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn
)

### Generating Module File
_generate_module_lisp(vgn
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vgn_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vgn_generate_messages vgn_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" NAME_WE)
add_dependencies(vgn_generate_messages_lisp _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_lisp _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_lisp _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" NAME_WE)
add_dependencies(vgn_generate_messages_lisp _vgn_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vgn_genlisp)
add_dependencies(vgn_genlisp vgn_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vgn_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn
)

### Generating Services
_generate_srv_nodejs(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn
)
_generate_srv_nodejs(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn
)
_generate_srv_nodejs(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn
)

### Generating Module File
_generate_module_nodejs(vgn
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(vgn_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(vgn_generate_messages vgn_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" NAME_WE)
add_dependencies(vgn_generate_messages_nodejs _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_nodejs _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_nodejs _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" NAME_WE)
add_dependencies(vgn_generate_messages_nodejs _vgn_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vgn_gennodejs)
add_dependencies(vgn_gennodejs vgn_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vgn_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
)

### Generating Services
_generate_srv_py(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
)
_generate_srv_py(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
)
_generate_srv_py(vgn
  "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
)

### Generating Module File
_generate_module_py(vgn
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vgn_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vgn_generate_messages vgn_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/msg/GraspConfig.msg" NAME_WE)
add_dependencies(vgn_generate_messages_py _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetMapCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_py _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/GetSceneCloud.srv" NAME_WE)
add_dependencies(vgn_generate_messages_py _vgn_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tom/dev_ws/thesis_ws/src/vgn/srv/PredictGrasps.srv" NAME_WE)
add_dependencies(vgn_generate_messages_py _vgn_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vgn_genpy)
add_dependencies(vgn_genpy vgn_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vgn_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vgn
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(vgn_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(vgn_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(vgn_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vgn
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(vgn_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(vgn_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(vgn_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vgn
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(vgn_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(vgn_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(vgn_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vgn
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(vgn_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(vgn_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(vgn_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vgn
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(vgn_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(vgn_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(vgn_generate_messages_py sensor_msgs_generate_messages_py)
endif()
