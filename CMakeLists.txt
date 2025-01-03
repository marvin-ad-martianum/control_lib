cmake_minimum_required(VERSION 3.10)
project(control_lib LANGUAGES CXX VERSION 0.1.0)

# ------------------------------------------------------------------------
# General settings
# ------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(DeePCpp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Use pkg-config to locate FFTW3
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)

# ------------------------------------------------------------------------
# Sources and Library
# ------------------------------------------------------------------------

# Collect source files
file(GLOB CONTROLLERS_SOURCES
     "${CMAKE_CURRENT_SOURCE_DIR}/src/controllers/*.cpp")
file(GLOB MODELS_SOURCES
     "${CMAKE_CURRENT_SOURCE_DIR}/src/models/*.cpp")
file(GLOB DATA_SOURCES
     "${CMAKE_CURRENT_SOURCE_DIR}/src/data_handling/*.cpp")

# Combine sources
set(SOURCES
    ${CONTROLLERS_SOURCES}
    ${MODELS_SOURCES}
    ${DATA_SOURCES}
)

# Fallback for empty sources
if(NOT SOURCES)
    message(FATAL_ERROR "No source files found for control_lib. Check source directories!")
endif()

# Create the shared library
add_library(control_lib SHARED ${SOURCES})

# Include directories for the library
target_include_directories(control_lib
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
      $<INSTALL_INTERFACE:include>
      ${EIGEN3_INCLUDE_DIRS}
)

target_compile_options(control_lib PRIVATE -Wall -Wextra -pedantic)


# ------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------
include(GNUInstallDirs)

install(
  TARGETS control_lib
  EXPORT  control_libTargets
  LIBRARY       DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE       DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME       DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY controllers/ models/ data_handling/ system_identification/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)

export(
  EXPORT control_libTargets
  FILE  "${CMAKE_CURRENT_BINARY_DIR}/control_libTargets.cmake"
  NAMESPACE control_lib::
)

install(
  EXPORT control_libTargets
  FILE control_libTargets.cmake
  NAMESPACE control_lib::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_lib
)

include(CMakePackageConfigHelpers)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/control_libConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/control_libConfig.cmake"
  @ONLY
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/control_libConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_lib
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/control_libConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/control_libConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/control_lib
)

# ------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Add GTest executable
    ament_add_gtest(control_lib_tests
        test/circular_buffer_test.cpp
    )

    # Dependencies for tests
    ament_target_dependencies(control_lib_tests
        Eigen3
        yaml-cpp
        jsoncpp
        DeePCpp
    )

    # Include FFTW3 via pkg-config
    target_include_directories(control_lib_tests PUBLIC ${FFTW3_INCLUDE_DIRS})
    target_link_libraries(control_lib_tests ${FFTW3_LIBRARIES})

    # Add include path for test sources
    target_include_directories(control_lib_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_include_directories(control_lib_tests PUBLIC ${EIGEN3_INCLUDE_DIRS})
endif()


# ------------------------------------------------------------------------
# Ament exports
# ------------------------------------------------------------------------
ament_export_targets(control_libTargets HAS_LIBRARY_TARGET)
ament_export_libraries(control_lib)
ament_export_include_directories(${CMAKE_INSTALL_INCLUDEDIR})
ament_export_dependencies(Eigen3 DeePCpp jsoncpp yaml-cpp)
ament_package()
