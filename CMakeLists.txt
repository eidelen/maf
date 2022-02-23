cmake_minimum_required(VERSION 3.0.0)
PROJECT(maf)

message("my cmake version " ${CMAKE_VERSION})

#---------------------------------------------------------------------
# library
#---------------------------------------------------------------------
add_subdirectory(lib)
add_subdirectory(guilib)
add_subdirectory(examples/guiexamples)
add_subdirectory(examples/constructive_sims)
add_subdirectory(examples/introduction)
