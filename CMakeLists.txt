cmake_minimum_required(VERSION 3.0.0)
PROJECT(maf)

message("my cmake version " ${CMAKE_VERSION})

#---------------------------------------------------------------------
# library
#---------------------------------------------------------------------
add_subdirectory(lib)
add_subdirectory(examples/claustrophobia)



