cmake_minimum_required(VERSION 4.1)

project(box2d-lite LANGUAGES CXX)

add_subdirectory(extern/glad)
add_subdirectory(extern/glfw)
add_subdirectory(extern/imgui)
add_subdirectory(base)
add_subdirectory(tests)
