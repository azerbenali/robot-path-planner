cmake_minimum_required(VERSION 3.16)
project(RobotPathPlanner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build raylib examples" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "Build raylib games" FORCE)
FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(robot_path_planner main.cpp PathPlanner.cpp)
target_link_libraries(robot_path_planner PRIVATE raylib)

if(MSVC)
    target_compile_options(robot_path_planner PRIVATE /W4 /permissive-)
else()
    target_compile_options(robot_path_planner PRIVATE -Wall -Wextra -Wpedantic)
endif()
