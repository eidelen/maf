cmake_minimum_required(VERSION 3.0)

PROJECT(constructivesims)

option(CONSTRUCTIVEENABLED  "Constructive simulations enabled" OFF)
IF(${CONSTRUCTIVEENABLED})
    MESSAGE(STATUS "Constructive simulations activated")
    add_subdirectory(claustrophobia)
    add_subdirectory(airdefence)
ENDIF()






