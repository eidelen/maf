cmake_minimum_required(VERSION 3.0)

PROJECT(airdefence)

option(ADENABLE  "Airdefence Example" OFF)
IF(${ADENABLE})
    MESSAGE(STATUS "Airdefence examples activated")

    # GUI
    find_package(Qt5Widgets REQUIRED)

    SET(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    include_directories( . ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

    add_executable(airdefence airdefencesim.h gui/glwidget.cpp gui/glwidget.h gui/main.cpp gui/window.h gui/window.cpp gui/adsimQt.h )
    target_link_libraries(airdefence Qt5::Widgets maflib guilib)
    target_compile_features(airdefence PRIVATE cxx_std_17 )

ENDIF()






