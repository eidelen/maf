cmake_minimum_required(VERSION 3.0)

PROJECT(claustrophobia)

option(CLENABLE  "Claustrophobia Examples" OFF)
IF(${CLENABLE})
    MESSAGE(STATUS "Claustrophobia examples activated")

    # GUI
    find_package(Qt5Widgets REQUIRED)

    SET(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    include_directories( . ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

    add_executable(claustrophobia clsimulation.h gui/glwidget.cpp gui/glwidget.h gui/main.cpp gui/window.h gui/window.cpp gui/clsimQt.h gui/resources.qrc )
    target_link_libraries(claustrophobia Qt5::Widgets maflib )
    target_compile_features(claustrophobia PRIVATE cxx_std_17 )

    # Shell
    add_executable(claustrophobiarunner clsimulation.h shell/main.cpp)
    target_link_libraries(claustrophobiarunner maflib )
    target_compile_features(claustrophobiarunner PRIVATE cxx_std_17 )

ENDIF()






