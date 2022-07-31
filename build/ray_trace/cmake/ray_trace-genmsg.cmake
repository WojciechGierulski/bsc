# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ray_trace: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iray_trace:/home/wgieruls/Desktop/bsc/src/ray_trace/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ray_trace_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" NAME_WE)
add_custom_target(_ray_trace_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ray_trace" "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" NAME_WE)
add_custom_target(_ray_trace_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ray_trace" "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ray_trace
)

### Generating Services
_generate_srv_cpp(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ray_trace
)

### Generating Module File
_generate_module_cpp(ray_trace
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ray_trace
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ray_trace_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ray_trace_generate_messages ray_trace_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" NAME_WE)
add_dependencies(ray_trace_generate_messages_cpp _ray_trace_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" NAME_WE)
add_dependencies(ray_trace_generate_messages_cpp _ray_trace_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ray_trace_gencpp)
add_dependencies(ray_trace_gencpp ray_trace_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ray_trace_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ray_trace
)

### Generating Services
_generate_srv_eus(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ray_trace
)

### Generating Module File
_generate_module_eus(ray_trace
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ray_trace
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ray_trace_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ray_trace_generate_messages ray_trace_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" NAME_WE)
add_dependencies(ray_trace_generate_messages_eus _ray_trace_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" NAME_WE)
add_dependencies(ray_trace_generate_messages_eus _ray_trace_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ray_trace_geneus)
add_dependencies(ray_trace_geneus ray_trace_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ray_trace_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ray_trace
)

### Generating Services
_generate_srv_lisp(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ray_trace
)

### Generating Module File
_generate_module_lisp(ray_trace
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ray_trace
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ray_trace_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ray_trace_generate_messages ray_trace_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" NAME_WE)
add_dependencies(ray_trace_generate_messages_lisp _ray_trace_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" NAME_WE)
add_dependencies(ray_trace_generate_messages_lisp _ray_trace_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ray_trace_genlisp)
add_dependencies(ray_trace_genlisp ray_trace_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ray_trace_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ray_trace
)

### Generating Services
_generate_srv_nodejs(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ray_trace
)

### Generating Module File
_generate_module_nodejs(ray_trace
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ray_trace
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ray_trace_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ray_trace_generate_messages ray_trace_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" NAME_WE)
add_dependencies(ray_trace_generate_messages_nodejs _ray_trace_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" NAME_WE)
add_dependencies(ray_trace_generate_messages_nodejs _ray_trace_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ray_trace_gennodejs)
add_dependencies(ray_trace_gennodejs ray_trace_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ray_trace_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ray_trace
)

### Generating Services
_generate_srv_py(ray_trace
  "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ray_trace
)

### Generating Module File
_generate_module_py(ray_trace
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ray_trace
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ray_trace_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ray_trace_generate_messages ray_trace_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/srv/ray_trace.srv" NAME_WE)
add_dependencies(ray_trace_generate_messages_py _ray_trace_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wgieruls/Desktop/bsc/src/ray_trace/msg/Triangle.msg" NAME_WE)
add_dependencies(ray_trace_generate_messages_py _ray_trace_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ray_trace_genpy)
add_dependencies(ray_trace_genpy ray_trace_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ray_trace_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ray_trace)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ray_trace
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ray_trace_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ray_trace_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ray_trace)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ray_trace
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ray_trace_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ray_trace_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ray_trace)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ray_trace
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ray_trace_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ray_trace_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ray_trace)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ray_trace
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ray_trace_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ray_trace_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ray_trace)
  install(CODE "execute_process(COMMAND \"/home/wgieruls/anaconda3/envs/bsc/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ray_trace\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ray_trace
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ray_trace_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ray_trace_generate_messages_py geometry_msgs_generate_messages_py)
endif()
