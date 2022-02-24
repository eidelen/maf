cmake_minimum_required(VERSION 3.0)

PROJECT(introductionexamples)

MESSAGE(STATUS "Building introduction examples")

add_executable(ex1app ex1.cpp)
target_link_libraries(ex1app maflib )
target_compile_features(ex1app PRIVATE cxx_std_17 )

add_executable(ex2app ex2.cpp)
target_link_libraries(ex2app maflib )
target_compile_features(ex2app PRIVATE cxx_std_17 )

