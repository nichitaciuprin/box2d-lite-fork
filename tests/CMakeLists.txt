project(Test1 LANGUAGES CXX)

add_executable(Test1 Test1.cpp)
target_include_directories(Test1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Test1 PUBLIC base glfw glad imgui ${OPENGL_LIBRARIES})

add_executable(Test2 Test2.cpp)
target_include_directories(Test2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Test2 PUBLIC base glfw glad imgui ${OPENGL_LIBRARIES})
