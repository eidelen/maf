cmake_minimum_required(VERSION 3.0)

PROJECT(claustrophobiarunner)

MESSAGE(STATUS "Claustrophobia constructive simulation activated")

include_directories( . ../../guiexamples/ )

add_executable(claustrophobiarunner ../../guiexamples/clsimulation.h main.cpp)
target_link_libraries(claustrophobiarunner maflib )
target_compile_features(claustrophobiarunner PRIVATE cxx_std_17 )







