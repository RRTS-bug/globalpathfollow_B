# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pathfollow: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipathfollow:/home/czp/globalPathfollow_B/src/pathfollow/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pathfollow_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" NAME_WE)
add_custom_target(_pathfollow_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pathfollow" "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pathfollow
  "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfollow
)

### Generating Services

### Generating Module File
_generate_module_cpp(pathfollow
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfollow
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pathfollow_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pathfollow_generate_messages pathfollow_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" NAME_WE)
add_dependencies(pathfollow_generate_messages_cpp _pathfollow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfollow_gencpp)
add_dependencies(pathfollow_gencpp pathfollow_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfollow_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pathfollow
  "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfollow
)

### Generating Services

### Generating Module File
_generate_module_eus(pathfollow
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfollow
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pathfollow_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pathfollow_generate_messages pathfollow_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" NAME_WE)
add_dependencies(pathfollow_generate_messages_eus _pathfollow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfollow_geneus)
add_dependencies(pathfollow_geneus pathfollow_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfollow_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pathfollow
  "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfollow
)

### Generating Services

### Generating Module File
_generate_module_lisp(pathfollow
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfollow
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pathfollow_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pathfollow_generate_messages pathfollow_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" NAME_WE)
add_dependencies(pathfollow_generate_messages_lisp _pathfollow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfollow_genlisp)
add_dependencies(pathfollow_genlisp pathfollow_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfollow_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pathfollow
  "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfollow
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pathfollow
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfollow
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pathfollow_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pathfollow_generate_messages pathfollow_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" NAME_WE)
add_dependencies(pathfollow_generate_messages_nodejs _pathfollow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfollow_gennodejs)
add_dependencies(pathfollow_gennodejs pathfollow_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfollow_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pathfollow
  "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfollow
)

### Generating Services

### Generating Module File
_generate_module_py(pathfollow
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfollow
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pathfollow_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pathfollow_generate_messages pathfollow_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/czp/globalPathfollow_B/src/pathfollow/msg/Traj.msg" NAME_WE)
add_dependencies(pathfollow_generate_messages_py _pathfollow_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfollow_genpy)
add_dependencies(pathfollow_genpy pathfollow_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfollow_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfollow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfollow
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(pathfollow_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(pathfollow_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(pathfollow_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pathfollow_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfollow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfollow
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(pathfollow_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(pathfollow_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(pathfollow_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pathfollow_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfollow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfollow
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(pathfollow_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(pathfollow_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(pathfollow_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pathfollow_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfollow)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfollow
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(pathfollow_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(pathfollow_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(pathfollow_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pathfollow_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfollow)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfollow\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfollow
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(pathfollow_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(pathfollow_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(pathfollow_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pathfollow_generate_messages_py std_msgs_generate_messages_py)
endif()
