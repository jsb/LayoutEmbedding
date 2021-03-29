cmake_minimum_required(VERSION 3.0)
project(layout-embedding)

set(LE_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/output" CACHE PATH "Output path for various files produced by the applications.")

set(CMAKE_CXX_STANDARD 17)

# Dependencies
# Warning: The order of these add_subdirectories matters, there are interdependencies.
set(GLOW_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}) # Viewer fonts will be placed here

if (NOT TARGET glfw)
    add_subdirectory(extern/glfw)
endif()

if (NOT TARGET typed-geometry)
    add_subdirectory(extern/typed-geometry)
endif()

if (NOT TARGET polymesh)
    add_subdirectory(extern/polymesh)
endif()

if (NOT TARGET glow)
    add_subdirectory(extern/glow)
endif()

if (NOT TARGET imgui)
    add_subdirectory(extern/imgui)
endif()

if (NOT TARGET glow-extras)
    add_subdirectory(extern/glow-extras)
endif()

if (NOT TARGET Eigen3::Eigen)
    add_subdirectory(extern/eigen-lean)
endif()

if (NOT TARGET cxxopts)
    add_subdirectory(extern/cxxopts)
endif()

find_package(OpenMP REQUIRED)

# LayoutEmbedding Library (library directory)
file(GLOB_RECURSE LE_LIBRARY_SOURCE_FILES "library/LayoutEmbedding/*.cc" "library/LayoutEmbedding/*.hh" "library/LayoutEmbedding/*.c" "library/LayoutEmbedding/*.h")
add_library(LayoutEmbedding ${LE_LIBRARY_SOURCE_FILES})
target_link_libraries(LayoutEmbedding PUBLIC imgui typed-geometry polymesh glow-extras eigen OpenMP::OpenMP_CXX)
target_include_directories(LayoutEmbedding PUBLIC library)
target_compile_definitions(LayoutEmbedding PUBLIC LE_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_compile_definitions(LayoutEmbedding PUBLIC LE_OUTPUT_PATH="${LE_OUTPUT_PATH}")
target_include_directories(LayoutEmbedding PRIVATE extern/libigl/include) # We use libigl header-only
target_link_libraries(LayoutEmbedding PRIVATE stdc++fs)

# Executable targets (apps directory)
file(GLOB_RECURSE LE_APP_SOURCE_FILES "apps/*.cc")
foreach(LE_APP_SOURCE_FILE ${LE_APP_SOURCE_FILES})
  get_filename_component(LE_APP_NAME ${LE_APP_SOURCE_FILE} NAME_WE)
  message("Executable target: ${LE_APP_NAME}")

  add_executable(${LE_APP_NAME} ${LE_APP_SOURCE_FILE})
  target_link_libraries(${LE_APP_NAME} PRIVATE LayoutEmbedding cxxopts::cxxopts)
endforeach()
