cmake_minimum_required(VERSION 2.8.3)

project(rclpy C)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

# NOTE(esteve): required for CMake-2.8 in Ubuntu 14.04
set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonInterp 3.4 REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
endif()

# enables using the Python package from the source space
# This creates a folder in ${CMAKE_CURRENT_BINARY_DIR}/rclpy which has all the
# as well as the compiled modules from the build space
# necessary Python code and C Python extensions for running tests.
configure_file("__init__.py.in" "rclpy/__init__.py" @ONLY)

ament_python_install_package(${PROJECT_NAME})

macro(set_properties _build_type)
  set_target_properties(${PROJECT_NAME}${target_suffix} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
    RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
    OUTPUT_NAME "_${PROJECT_NAME}${target_suffix}${PythonExtra_EXTENSION_SUFFIX}"
    SUFFIX "${PythonExtra_EXTENSION_EXTENSION}")
endmacro()

macro(target)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()
  if("${rmw_implementation} " STREQUAL "rmw_connext_dynamic_cpp ")
    message(STATUS "Skipping rclpy for '${rmw_implementation}'")
    return()
  endif()
  if("${rmw_implementation} " STREQUAL "rmw_fastrtps_cpp ")
    message(STATUS "Skipping rclpy for '${rmw_implementation}'")
    return()
  endif()

  add_library(
    ${PROJECT_NAME}${target_suffix}
    SHARED src/rclpy/_rclpy.c
  )
  target_compile_definitions(${PROJECT_NAME}${target_suffix}
    PRIVATE "RMW_IMPLEMENTATION_SUFFIX=${target_suffix}")

  set_properties("")
  if(WIN32)
    set_properties("_DEBUG")
    set_properties("_MINSIZEREL")
    set_properties("_RELEASE")
    set_properties("_RELWITHDEBINFO")
  endif()

  target_link_libraries(${PROJECT_NAME}${target_suffix}
    ${PythonExtra_LIBRARIES}
  )

  ament_target_dependencies(${PROJECT_NAME}${target_suffix}
    "rcl${target_suffix}"
  )

  target_include_directories(${PROJECT_NAME}${target_suffix}
    PUBLIC
    ${PythonExtra_INCLUDE_DIRS}
  )

  # make an entry in the ament index for each rmw implementation for which
  # a rclpy Python extension is built
  # this is used in the Python code to avoid trying to import a Python
  # extension for rmw implementations which do not have one
  ament_index_register_resource("rmw_python_extension"
    PACKAGE_NAME ${rmw_implementation}
  )

  install(TARGETS ${PROJECT_NAME}${target_suffix}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
endmacro()

call_for_each_rmw_implementation(target)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  rosidl_generator_py_get_typesupports(_typesupport_impls)
  ament_index_get_prefix_path(ament_prefix_path_list)
  # Do this even on Windows, because the Python run_test.py will handle the : vs ; issues.
  string(REPLACE ";" ":" ament_prefix_path "${ament_prefix_path_list}")
  if(WIN32)
    string(REPLACE "\\" "/" ament_prefix_path "${ament_prefix_path}")
  endif()
  if(NOT "${_typesupport_impls} " STREQUAL " ")
    ament_add_nose_test(rclpytests test
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      ENV AMENT_PREFIX_PATH=${ament_prefix_path}
    )
  endif()
endif()

ament_package()
