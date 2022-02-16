cmake_minimum_required(VERSION 3.0)

PROJECT(airdefencerunner)

MESSAGE(STATUS "Airdefence constructive simulation activated")

include_directories( . ../../guiexamples/ )

add_executable(airdefencerunner ../../guiexamples/airdefencesim.h main.cpp)
target_link_libraries(airdefencerunner maflib )
target_compile_features(airdefencerunner PRIVATE cxx_std_17 )







