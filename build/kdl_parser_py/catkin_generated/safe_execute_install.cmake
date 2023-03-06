execute_process(COMMAND "/home/tom/dev_ws/thesis_ws/build/kdl_parser_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/tom/dev_ws/thesis_ws/build/kdl_parser_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
