cmake_minimum_required(VERSION 3.0)
project(sheeted-tracing)

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

#set(LIBIGL_USE_STATIC_LIBRARY OFF)
#set(LIBIGL_WITH_COMISO OFF)
#set(LIBIGL_WITH_EMBREE OFF)
#set(LIBIGL_WITH_OPENGL OFF)
#add_subdirectory(extern/libigl)

# Main executable
file(GLOB_RECURSE SOURCES "src/*.cc" "src/*.hh")
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC imgui typed-geometry polymesh glow-extras eigen)
target_include_directories(${PROJECT_NAME} PRIVATE extern/libigl/include)
