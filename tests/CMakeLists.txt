project(Test1 LANGUAGES CXX)

set(SAMPLE_SOURCE_FILES main.cpp)

add_executable(Test1 ${SAMPLE_SOURCE_FILES})
target_include_directories(Test1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Test1 PUBLIC base glfw glad imgui ${OPENGL_LIBRARIES})
