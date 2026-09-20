project(config C)

set(CONFIG_SOURCE_FILES src/Config.cpp)

set(CONFIG_HEADER_FILES
    include/Config.h
    include/Core.h
    include/MathUtils.h
    include/Window.h)

add_library(config STATIC ${CONFIG_SOURCE_FILES} ${CONFIG_HEADER_FILES})
target_include_directories(config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
