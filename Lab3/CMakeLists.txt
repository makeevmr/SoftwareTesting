cmake_minimum_required(VERSION 3.20)

project(Lab3 LANGUAGES CXX)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-13")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# use target_compile_options(<target_name> PUBLIC <flags_list>) for each translation unit or use approach below
set(GCC_ADDITIONAL_WARNINGS_COMPILE_FLAGS "-Werror -Wall -Wextra -Wpedantic -Wcast-align -Wcast-qual -Wconversion -Wctor-dtor-privacy -Wenum-compare -Wfloat-equal -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wsign-conversion -Wsign-promo")

set(GCC_LINK_FLAGS "")

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)

find_package(GTest REQUIRED)

add_executable(lab3_tests)

target_link_libraries(lab3_tests PRIVATE 
    GTest::GTest
    bgl_tests
    merge
    graph_validation
)

add_test(lab3_gtests lab3_tests)