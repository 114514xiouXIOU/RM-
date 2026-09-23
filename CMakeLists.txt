cmake_minimum_required(VERSION 3.16)

project(reproject LANGUAGES CXX)

# C++17（作业要求）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 核心实现
add_library(reproject_core STATIC
    src/camera.cpp
    src/reprojection.cpp
)
target_include_directories(reproject_core PUBLIC include)

# 可执行文件
add_executable(reproject src/main.cpp)
target_link_libraries(reproject PRIVATE reproject_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(reproject_core PRIVATE -Wall -Wextra)
    target_compile_options(reproject PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(reproject_core PRIVATE /W4)
    target_compile_options(reproject PRIVATE /W4)
endif()
