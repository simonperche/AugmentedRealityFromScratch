cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(APP_NAME AR)

project(${APP_NAME})

if (CMAKE_COMPILER_IS_GNUCC)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(COMPILATION_FLAGS
                -Wall
                -Wextra
                -Wshadow
                -pedantic
                -Wnon-virtual-dtor
                -Wold-style-cast
                -Wcast-align
                -Wunused
                -Woverloaded-virtual
                -Wpedantic
                -Wconversion
                -Wsign-conversion
                -Wmisleading-indentation
                -Wduplicated-cond
                -Wduplicated-branches
                -Wlogical-op
                -Wnull-dereference
                -Wuseless-cast
                -Wdouble-promotion
                -Wformat=2)
    else()
        set(COMPILATION_FLAGS
                -DNDEBUG
                -Wall
                -Wextra
                -Wshadow
                -pedantic
                -Wnon-virtual-dtor
                -Wold-style-cast
                -Wcast-align
                -Wunused
                -Woverloaded-virtual
                -Wpedantic
                -Wconversion
                -Wsign-conversion
                -Wmisleading-indentation
                -Wduplicated-cond
                -Wduplicated-branches
                -Wlogical-op
                -Wnull-dereference
                -Wuseless-cast
                -Wdouble-promotion
                -Wformat=2)
    endif()
elseif (MSVC)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(COMPILATION_FLAGS
                /permissive
                /w14242
                /w14254
                /w14263
                /w14265
                /w14287
                /we4289
                /w14296
                /w14311
                /w14545
                /w14546
                /w14547
                /w14549
                /w14555
                /w14619
                /w14640
                /w14826
                /w14928)
    else()
        set(COMPILATION_FLAGS
                /permissive
                /w14242
                /w14254
                /w14263
                /w14265
                /w14287
                /we4289
                /w14296
                /w14311
                /w14545
                /w14546
                /w14547
                /w14549
                /w14555
                /w14619
                /w14640
                /w14826
                /w14928)
    endif()
endif ()

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES
        src/main.cpp
        src/Video.cpp
        src/Tracking.cpp
        src/Utils.cpp
        src/OBJLoader.cpp
        src/ARTag.cpp
        src/TagDetection.cpp
        src/Camera.cpp
        src/Object.cpp
        src/Scene.cpp
        src/Renderer.cpp
        src/CommandLineParser.cpp)

set(HEADERS_FILES
        headers/Video.hpp
        headers/Tracking.hpp
        headers/Utils.hpp
        headers/OBJLoader.hpp
        headers/ARTag.hpp
        headers/TagDetection.hpp
        headers/Camera.hpp
        headers/Object.hpp
        headers/Scene.hpp
        headers/Renderer.hpp
        headers/exceptions.hpp
        headers/CommandLineParser.hpp)

add_executable(${APP_NAME} ${SOURCE_FILES} ${HEADERS_FILES})

target_link_libraries(${APP_NAME} ${OpenCV_LIBS})

target_compile_options(${APP_NAME} PUBLIC ${COMPILATION_FLAGS})