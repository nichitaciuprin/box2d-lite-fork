project(base C)

set(CONFIG_SOURCE_FILES src/Config.cpp)

set(CONFIG_HEADER_FILES
    include/Config.h
    include/Core.h
    include/MathUtils.h
    include/Physics.h
    include/Physics2.h
    include/Window.h)

add_library(base STATIC ${CONFIG_SOURCE_FILES} ${CONFIG_HEADER_FILES})
target_include_directories(base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
