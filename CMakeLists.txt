cmake_minimum_required(VERSION 2.8.3)

project(rclpy)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(include)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
endif()

# NOTE(esteve): required for CMake-2.8 in Ubuntu 14.04
set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonInterp 3.4 REQUIRED)

find_program(PYTHON_CONFIG_EXECUTABLE NAMES "python3-config")
if(NOT DEFINED PYTHON_CONFIG_EXECUTABLE)
  message(FATAL_ERROR "Cannot find python3-config executable")
endif()

if(APPLE)
  if(NOT DEFINED PYTHON_INCLUDE_DIRS)
    execute_process(
      COMMAND
      "${PYTHON_CONFIG_EXECUTABLE}"
      "--includes"
      OUTPUT_VARIABLE _output
      RESULT_VARIABLE _result
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _result EQUAL 0)
      message(FATAL_ERROR
        "execute_process(${PYTHON_CONFIG_EXECUTABLE} --includes) returned "
        "error code ${_result}")
    endif()

    string(REPLACE " " ";" _output_list ${_output})

    foreach(_includedir ${_output_list})
      string(SUBSTRING "${_includedir}" 2 -1 _includedir)
      list(APPEND PYTHON_INCLUDE_DIRS "${_includedir}")
    endforeach()
  endif()
  set(PYTHON_INCLUDE_DIRS
      ${PYTHON_INCLUDE_DIRS}
      CACHE INTERNAL
      "The paths to the Python include directories.")
  message(STATUS "Using PYTHON_INCLUDE_DIRS: ${PYTHON_INCLUDE_DIRS}")

#  if(NOT DEFINED PYTHON_LIBRARIES)
    execute_process(
      COMMAND
      "${PYTHON_CONFIG_EXECUTABLE}"
      "--ldflags"
      OUTPUT_VARIABLE _output
      RESULT_VARIABLE _result
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _result EQUAL 0)
      message(FATAL_ERROR
        "execute_process(${PYTHON_CONFIG_EXECUTABLE} --ldflags) returned "
        "error code ${_result}")
    endif()

    string(REPLACE " " ";" _output_list "${_output}")
    set(PYTHON_LIBRARIES
      ""
      CACHE INTERNAL
      "The libraries that need to be linked against for Python extensions.")

    set(_library_paths "")
    foreach(_item ${_output_list})
      string(REGEX MATCH "-L(.*)" _regex_match ${_item})
      if(NOT "${_regex_match} " STREQUAL " ")
        string(SUBSTRING "${_regex_match}" 2 -1 _library_path)
        list(APPEND _library_paths "${_library_path}")
      endif()
    endforeach()

    set(_python_version_no_dots
      "${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
    set(_python_version
      "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

    find_library(PYTHON_LIBRARY
      NAMES
      python${_python_version_no_dots}
      python${_python_version}mu
      python${_python_version}m
      python${_python_version}u
      python${_python_version}
      PATHS
      ${_library_paths}
      NO_SYSTEM_ENVIRONMENT_PATH
    )
 # endif()

  set(PYTHON_LIBRARIES "${PYTHON_LIBRARY}")
  message(STATUS "Using PYTHON_LIBRARIES: ${PYTHON_LIBRARIES}")
else()
  find_package(PythonLibs 3.4 REQUIRED)
endif()

if(NOT DEFINED PYTHON_SOABI)
  set(_python_code
    "from sysconfig import get_config_var"
    "print(get_config_var('SOABI'))"
  )
  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}"
    "-c"
    "${_python_code}"
    OUTPUT_VARIABLE _output
    RESULT_VARIABLE _result
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR
      "execute_process(${PYTHON_EXECUTABLE} -c '${_python_code}') returned "
      "error code ${_result}")
  endif()

  set(PYTHON_SOABI
    "${_output}"
    CACHE INTERNAL
    "The SOABI suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/.")
endif()

if(NOT DEFINED PYTHON_MULTIARCH)
  set(_python_code
    "from sysconfig import get_config_var"
    "print(get_config_var('MULTIARCH'))"
  )
  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}"
    "-c"
    "${_python_code}"
    OUTPUT_VARIABLE _output
    RESULT_VARIABLE _result
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _result EQUAL 0)
    message(FATAL_ERROR
      "execute_process(${PYTHON_EXECUTABLE} -c '${_python_code}') returned "
      "error code ${_result}")
  endif()

  set(PYTHON_MULTIARCH
    "${_output}"
    CACHE INTERNAL
    "The MULTIARCH suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/.")
endif()

if("${PYTHON_SOABI} " STREQUAL " ")
  set(PYTHON_EXTENSION_SUFFIX
    ""
    CACHE INTERNAL
    "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
  )
else()
  if("${PYTHON_MULTIARCH} " STREQUAL " ")
    set(PYTHON_EXTENSION_SUFFIX
      ".${PYTHON_SOABI}"
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  else()
    set(PYTHON_EXTENSION_SUFFIX
      ".${PYTHON_SOABI}-${PYTHON_MULTIARCH}"
      CACHE INTERNAL
      "The full suffix for Python native extensions. See PEP-3149: https://www.python.org/dev/peps/pep-3149/."
    )
  endif()
endif()

macro(target)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()
  if("${rmw_implementation} " STREQUAL "rmw_connext_dynamic_cpp ")
    message(STATUS "Skipping tests for '${rmw_implementation}'")
    return()
  endif()
  if("${rmw_implementation} " STREQUAL "rmw_fastrtps_cpp ")
    message(STATUS "Skipping tests for '${rmw_implementation}'")
    return()
  endif()

  configure_file(
    src/rclpy/_rclpy.c.in
    src/rclpy/_rclpy${target_suffix}.c
  )

  add_library(
    ${PROJECT_NAME}${target_suffix}
    SHARED src/rclpy/_rclpy${target_suffix}.c
  )

#  get_rmw_typesupport(_typesupport_impls ${rmw_implementation})

if(WIN32)
  set(PYTHON_EXTENSION_EXTENSION ".pyd")
else()
# Also use .so for OSX, not dylib
  set(PYTHON_EXTENSION_EXTENSION ".so")
endif()

  set_target_properties(${PROJECT_NAME}${target_suffix} PROPERTIES
    PREFIX ""
    OUTPUT_NAME "_${PROJECT_NAME}${target_suffix}${PYTHON_EXTENSION_SUFFIX}"
    SUFFIX "${PYTHON_EXTENSION_EXTENSION}"
  )

    target_link_libraries(
    ${PROJECT_NAME}${target_suffix}
      ${PYTHON_LIBRARIES}
    )

  ament_target_dependencies(${PROJECT_NAME}${target_suffix}
    "rcl${target_suffix}"
  )

  target_include_directories(${PROJECT_NAME}${target_suffix}
    PUBLIC
    ${PYTHON_INCLUDE_DIRS}
  )

  install(
    TARGETS ${PROJECT_NAME}${target_suffix}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endmacro()

call_for_each_rmw_implementation(target)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()
