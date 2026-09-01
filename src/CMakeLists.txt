set(BOX2D_SOURCE_FILES Config.cpp)

set(BOX2D_HEADER_FILES
    ../include/Config.h
	../include/MathUtils.h
	../include/World.h)

add_library(box2d-lite STATIC ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})
target_include_directories(box2d-lite PUBLIC ../include)