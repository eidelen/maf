cmake_minimum_required(VERSION 3.0)

PROJECT(guiexamples)

option(GUIEXENABLED  "GUI Examples" OFF)
IF(${GUIEXENABLED})
    MESSAGE(STATUS "GUI examples activated")

    # GUI
    find_package(Qt5Widgets REQUIRED)

    SET(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    include_directories( . ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

    add_executable(guiexamples
        airdefencesim.h clsimulation.h
        gui/glwidget.cpp gui/glwidget.h gui/main.cpp gui/window.h
        gui/window.cpp gui/adsimQt.h gui/clsimQt.h gui/simQt.h )
    target_link_libraries(guiexamples Qt5::Widgets maflib guilib)
    target_compile_features(guiexamples PRIVATE cxx_std_17 )

ENDIF()






