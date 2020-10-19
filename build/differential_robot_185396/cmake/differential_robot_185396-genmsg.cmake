# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "differential_robot_185396: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idifferential_robot_185396:/home/alekal/catkin_ws/src/differential_robot_185396/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(differential_robot_185396_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" NAME_WE)
add_custom_target(_differential_robot_185396_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "differential_robot_185396" "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(differential_robot_185396
  "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_robot_185396
)

### Generating Services

### Generating Module File
_generate_module_cpp(differential_robot_185396
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_robot_185396
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(differential_robot_185396_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(differential_robot_185396_generate_messages differential_robot_185396_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" NAME_WE)
add_dependencies(differential_robot_185396_generate_messages_cpp _differential_robot_185396_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_robot_185396_gencpp)
add_dependencies(differential_robot_185396_gencpp differential_robot_185396_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_robot_185396_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(differential_robot_185396
  "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_robot_185396
)

### Generating Services

### Generating Module File
_generate_module_eus(differential_robot_185396
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_robot_185396
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(differential_robot_185396_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(differential_robot_185396_generate_messages differential_robot_185396_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" NAME_WE)
add_dependencies(differential_robot_185396_generate_messages_eus _differential_robot_185396_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_robot_185396_geneus)
add_dependencies(differential_robot_185396_geneus differential_robot_185396_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_robot_185396_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(differential_robot_185396
  "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_robot_185396
)

### Generating Services

### Generating Module File
_generate_module_lisp(differential_robot_185396
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_robot_185396
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(differential_robot_185396_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(differential_robot_185396_generate_messages differential_robot_185396_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" NAME_WE)
add_dependencies(differential_robot_185396_generate_messages_lisp _differential_robot_185396_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_robot_185396_genlisp)
add_dependencies(differential_robot_185396_genlisp differential_robot_185396_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_robot_185396_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(differential_robot_185396
  "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_robot_185396
)

### Generating Services

### Generating Module File
_generate_module_nodejs(differential_robot_185396
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_robot_185396
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(differential_robot_185396_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(differential_robot_185396_generate_messages differential_robot_185396_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" NAME_WE)
add_dependencies(differential_robot_185396_generate_messages_nodejs _differential_robot_185396_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_robot_185396_gennodejs)
add_dependencies(differential_robot_185396_gennodejs differential_robot_185396_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_robot_185396_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(differential_robot_185396
  "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_robot_185396
)

### Generating Services

### Generating Module File
_generate_module_py(differential_robot_185396
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_robot_185396
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(differential_robot_185396_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(differential_robot_185396_generate_messages differential_robot_185396_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alekal/catkin_ws/src/differential_robot_185396/msg/counter_message.msg" NAME_WE)
add_dependencies(differential_robot_185396_generate_messages_py _differential_robot_185396_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(differential_robot_185396_genpy)
add_dependencies(differential_robot_185396_genpy differential_robot_185396_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS differential_robot_185396_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_robot_185396)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/differential_robot_185396
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(differential_robot_185396_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_robot_185396)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/differential_robot_185396
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(differential_robot_185396_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_robot_185396)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/differential_robot_185396
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(differential_robot_185396_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_robot_185396)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/differential_robot_185396
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(differential_robot_185396_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_robot_185396)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_robot_185396\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/differential_robot_185396
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(differential_robot_185396_generate_messages_py std_msgs_generate_messages_py)
endif()
