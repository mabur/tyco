# Top-Level CMakeList.txt
cmake_minimum_required(VERSION 3.4)
enable_language(CXX)
add_definitions(-std=c++11)

include_directories(include)
file(GLOB tyco_header_files "include/*.hpp")

PROJECT(tyco)
add_executable(test_tyco tests/main.cpp ${tyco_header_files})
