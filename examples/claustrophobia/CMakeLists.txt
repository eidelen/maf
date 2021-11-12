cmake_minimum_required(VERSION 3.0)

PROJECT(claustrophobia)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x")

option(CLENABLE  "Claustrophobia Example" OFF)
IF(${CLENABLE})
    MESSAGE(STATUS "Claustrophobia example activated")
    FILE(GLOB  CL_EX_INC       *.h)
    FILE(GLOB  CL_EX_SRC       *.cpp)

    find_package(Qt5Widgets REQUIRED)

    SET(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    include_directories( .  ${CL_EX_INC} ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} )
    add_executable(claustrophobia ${CL_EX_SRC} resources.qrc )
    #target_link_libraries(claustrophobia Qt5::Widgets pthread maflib )
    target_link_libraries(claustrophobia Qt5::Widgets maflib )

ENDIF()






