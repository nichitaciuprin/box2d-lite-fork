project(tests LANGUAGES CXX)

set(SAMPLE_SOURCE_FILES main.cpp)

add_executable(tests ${SAMPLE_SOURCE_FILES})
target_include_directories(tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests PUBLIC config glfw glad imgui ${OPENGL_LIBRARIES})
