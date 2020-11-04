cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 20)

set(APP_NAME AR)

project(${APP_NAME})

set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES
        src/main.cpp)

#list(APPEND HEADERS_FILES
#        "")

add_executable(${APP_NAME} ${SOURCE_FILES}) # ${HEADERS_FILES})
target_link_libraries(${APP_NAME} ${OpenCV_LIBS})