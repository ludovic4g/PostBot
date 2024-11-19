# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "postbot: 3 messages, 2 services")

set(MSG_I_FLAGS "-Ipostbot:/home/ludoludo/postbot/src/postbot/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(postbot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" NAME_WE)
add_custom_target(_postbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "postbot" "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" ""
)

get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" NAME_WE)
add_custom_target(_postbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "postbot" "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" ""
)

get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" NAME_WE)
add_custom_target(_postbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "postbot" "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" ""
)

get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" NAME_WE)
add_custom_target(_postbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "postbot" "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" ""
)

get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" NAME_WE)
add_custom_target(_postbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "postbot" "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
)
_generate_msg_cpp(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
)
_generate_msg_cpp(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
)

### Generating Services
_generate_srv_cpp(postbot
  "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
)
_generate_srv_cpp(postbot
  "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
)

### Generating Module File
_generate_module_cpp(postbot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(postbot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(postbot_generate_messages postbot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_cpp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_cpp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" NAME_WE)
add_dependencies(postbot_generate_messages_cpp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" NAME_WE)
add_dependencies(postbot_generate_messages_cpp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" NAME_WE)
add_dependencies(postbot_generate_messages_cpp _postbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(postbot_gencpp)
add_dependencies(postbot_gencpp postbot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS postbot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
)
_generate_msg_eus(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
)
_generate_msg_eus(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
)

### Generating Services
_generate_srv_eus(postbot
  "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
)
_generate_srv_eus(postbot
  "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
)

### Generating Module File
_generate_module_eus(postbot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(postbot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(postbot_generate_messages postbot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_eus _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_eus _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" NAME_WE)
add_dependencies(postbot_generate_messages_eus _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" NAME_WE)
add_dependencies(postbot_generate_messages_eus _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" NAME_WE)
add_dependencies(postbot_generate_messages_eus _postbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(postbot_geneus)
add_dependencies(postbot_geneus postbot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS postbot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
)
_generate_msg_lisp(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
)
_generate_msg_lisp(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
)

### Generating Services
_generate_srv_lisp(postbot
  "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
)
_generate_srv_lisp(postbot
  "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
)

### Generating Module File
_generate_module_lisp(postbot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(postbot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(postbot_generate_messages postbot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_lisp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_lisp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" NAME_WE)
add_dependencies(postbot_generate_messages_lisp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" NAME_WE)
add_dependencies(postbot_generate_messages_lisp _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" NAME_WE)
add_dependencies(postbot_generate_messages_lisp _postbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(postbot_genlisp)
add_dependencies(postbot_genlisp postbot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS postbot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
)
_generate_msg_nodejs(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
)
_generate_msg_nodejs(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
)

### Generating Services
_generate_srv_nodejs(postbot
  "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
)
_generate_srv_nodejs(postbot
  "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
)

### Generating Module File
_generate_module_nodejs(postbot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(postbot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(postbot_generate_messages postbot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_nodejs _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_nodejs _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" NAME_WE)
add_dependencies(postbot_generate_messages_nodejs _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" NAME_WE)
add_dependencies(postbot_generate_messages_nodejs _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" NAME_WE)
add_dependencies(postbot_generate_messages_nodejs _postbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(postbot_gennodejs)
add_dependencies(postbot_gennodejs postbot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS postbot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
)
_generate_msg_py(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
)
_generate_msg_py(postbot
  "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
)

### Generating Services
_generate_srv_py(postbot
  "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
)
_generate_srv_py(postbot
  "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
)

### Generating Module File
_generate_module_py(postbot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(postbot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(postbot_generate_messages postbot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BallInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_py _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxInfo.msg" NAME_WE)
add_dependencies(postbot_generate_messages_py _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/msg/BoxGoal.msg" NAME_WE)
add_dependencies(postbot_generate_messages_py _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/reset_boxes.srv" NAME_WE)
add_dependencies(postbot_generate_messages_py _postbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ludoludo/postbot/src/postbot/srv/spawn_ball.srv" NAME_WE)
add_dependencies(postbot_generate_messages_py _postbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(postbot_genpy)
add_dependencies(postbot_genpy postbot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS postbot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/postbot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(postbot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/postbot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(postbot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/postbot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(postbot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/postbot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(postbot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/postbot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(postbot_generate_messages_py std_msgs_generate_messages_py)
endif()
