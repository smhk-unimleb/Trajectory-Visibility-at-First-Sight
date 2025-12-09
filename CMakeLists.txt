cmake_minimum_required(VERSION 3.30)
project(TrajVisFirstsight VERSION 1.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Academic/Rigorous Warning Level
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(Catch2)

add_library(tv_core
        src/geometry.cpp src/geometry.h
        src/math_solver.cpp src/math_solver.h
        src/first_sight.cpp src/first_sight.h
        # THEOREM 1 FILES:
        src/linear_shortest_path.cpp src/linear_shortest_path.h
        src/splinegon.cpp src/splinegon.h
        src/linear_shortest_path.h
)
target_include_directories(tv_core PUBLIC src)

add_executable(run_tests
        tests/test_main.cpp
)
target_link_libraries(run_tests PRIVATE tv_core Catch2::Catch2WithMain)