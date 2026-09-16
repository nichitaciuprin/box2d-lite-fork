project(samples LANGUAGES CXX)

set (SAMPLE_SOURCE_FILES main.cpp)

add_executable(samples ${SAMPLE_SOURCE_FILES})
target_include_directories(samples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(samples PUBLIC box2d-lite glfw glad imgui ${OPENGL_LIBRARIES})

