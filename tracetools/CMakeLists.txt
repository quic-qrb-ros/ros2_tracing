cmake_minimum_required(VERSION 3.5)
project(tracetools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -O2)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(/W4)
endif()

find_package(ament_cmake_ros REQUIRED)

if(WIN32)
  set(DISABLED_DEFAULT ON)
else()
  set(DISABLED_DEFAULT OFF)
endif()
option(TRACETOOLS_DISABLED "Explicitly disable support for tracing" ${DISABLED_DEFAULT})
option(TRACETOOLS_NO_RDYNAMIC "Disable export of -rdynamic link flag" OFF)
option(TRACETOOLS_STATUS_CHECKING_TOOL "Enable the status checking tool" ON)
option(TRACETOOLS_PERFETTO_TRACING_TOOL "Enable the perfetto tracing tool" OFF)

if(NOT TRACETOOLS_DISABLED)
  set(TRACETOOLS_PERFETTO_ENABLED TRUE)
  set(TRACETOOLS_STATUS_CHECKING_TOOL ON)
  set(TRACETOOLS_PERFETTO_TRACING_TOOL ON)
  message("Perfetto found: tracing enabled")
  message ("TRACETOOLS_DISABLED = ${TRACETOOLS_DISABLED}")
  message ("TRACETOOLS_PERFETTO_ENABLED = ${TRACETOOLS_PERFETTO_ENABLED}")
  message ("TRACETOOLS_STATUS_CHECKING_TOOL = ${TRACETOOLS_STATUS_CHECKING_TOOL}")
  message ("TRACETOOLS_PERFETTO_TRACING_TOOL = ${TRACETOOLS_PERFETTO_TRACING_TOOL}")
endif()

# Store configuration variables for runtime use
#   TRACETOOLS_DISABLED
#   TRACETOOLS_PERFETTO_ENABLED
configure_file(include/${PROJECT_NAME}/config.h.in include/${PROJECT_NAME}/config.h)

include_directories(
  include/
)

# Tracetools lib
set(SOURCES
  src/tracetools.cpp
  src/utils.cpp
)
set(HEADERS
  include/${PROJECT_NAME}/tracetools.h
  include/${PROJECT_NAME}/utils.hpp
  include/${PROJECT_NAME}/visibility_control.hpp
)

if(TRACETOOLS_PERFETTO_ENABLED)
  # We only need these if we're using perfetto
  list(APPEND HEADERS
    include/${PROJECT_NAME}/perfetto_trace.h
    include/perfetto_sdk/perfetto.h
  )
  add_library(perfetto_lib SHARED include/perfetto_sdk/perfetto.cc)
endif()

# Copy select headers to the actual include/ directory that we will use and export
foreach(_header ${HEADERS})
  configure_file(
    ${PROJECT_SOURCE_DIR}/${_header}
    ${PROJECT_BINARY_DIR}/${_header}
    COPYONLY
  )
endforeach()

add_library(${PROJECT_NAME} ${SOURCES})

if(TRACETOOLS_PERFETTO_ENABLED)
  target_link_libraries(${PROJECT_NAME} perfetto_lib dl)
  if(NOT TRACETOOLS_NO_RDYNAMIC)
    target_link_libraries(${PROJECT_NAME} "-rdynamic")
  endif()

  install(
    TARGETS perfetto_lib
    EXPORT ${PROJECT_NAME}_export
  )
endif()
if(WIN32)
  # Causes the visibility macros to use dllexport rather than dllimport
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(${PROJECT_NAME} PRIVATE "TRACETOOLS_BUILDING_DLL")
endif()

# Only use output/binary include directory
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

if(TRACETOOLS_STATUS_CHECKING_TOOL)
  # Lib for status checking
  add_library(${PROJECT_NAME}_status
    src/status.c
  )
  target_link_libraries(${PROJECT_NAME}_status
    ${PROJECT_NAME}
  )
  install(TARGETS
    ${PROJECT_NAME}_status
    DESTINATION lib
  )
  list(APPEND HEADERS
    include
  )
  # Status checking tool
  add_executable(status
    src/status_tool.c
  )
  target_link_libraries(status
    ${PROJECT_NAME}
    ${PROJECT_NAME}_status
  )
  install(TARGETS
    status
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TRACETOOLS_PERFETTO_TRACING_TOOL)
  # Perfetto tracing tool
  add_executable(perfetto_trace
    src/perfetto_trace.cpp
  )
  target_link_libraries(perfetto_trace
    ${PROJECT_NAME}
  )
  install(TARGETS
    perfetto_trace
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY ${PROJECT_BINARY_DIR}/include/
  DESTINATION include
)

ament_export_include_directories(include)
if(TRACETOOLS_PERFETTO_ENABLED)
  ament_export_libraries(${PROJECT_NAME} ${PERFETTO_LIBRARIES})
  # Export -rdynamic for downstream packages using classic CMake variables
  if(NOT TRACETOOLS_NO_RDYNAMIC)
    ament_export_link_flags("-rdynamic")
  endif()
else()
  ament_export_libraries(${PROJECT_NAME})
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

target_compile_definitions(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_VERSION="${${PROJECT_NAME}_VERSION}")
