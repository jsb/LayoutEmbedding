cmake_minimum_required(VERSION 3.0)
project(layout-embedding)

set(CMAKE_CXX_STANDARD 17)

# Dependencies
# Warning: The order of these add_subdirectories matters, there are interdependencies.
set(GLOW_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}) # Viewer fonts will be placed here
add_subdirectory(extern/glfw)
add_subdirectory(extern/typed-geometry)
add_subdirectory(extern/polymesh)
add_subdirectory(extern/glow)
add_subdirectory(extern/imgui)
add_subdirectory(extern/glow-extras)
add_subdirectory(extern/eigen-lean)

# LayoutEmbedding Library (library directory)
file(GLOB_RECURSE LE_SOURCES "library/LayoutEmbedding/*.cc" "library/LayoutEmbedding/*.hh")
add_library(LayoutEmbedding ${LE_SOURCES})
target_link_libraries(LayoutEmbedding PUBLIC imgui typed-geometry polymesh glow-extras eigen)
target_include_directories(LayoutEmbedding PUBLIC library)
target_compile_definitions(LayoutEmbedding PUBLIC LE_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_include_directories(LayoutEmbedding PRIVATE extern/libigl/include) # We use libigl header-only

# Executable targets (applications directory)
file(GLOB APP_SOURCE_FILES "applications/*.cc")
foreach(APP_SOURCE_FILE ${APP_SOURCE_FILES})
  get_filename_component(APP_NAME ${APP_SOURCE_FILE} NAME_WE)
  message("Executable target: ${APP_NAME}")

  add_executable(${APP_NAME} ${APP_SOURCE_FILE})
  target_link_libraries(${APP_NAME} PRIVATE LayoutEmbedding)
endforeach()
