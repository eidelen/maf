cmake_minimum_required(VERSION 3.0)

PROJECT(guilib)

option(GUIENABLE  "Gui Library" OFF)
IF(${GUIENABLE})
    MESSAGE(STATUS "Gui Library Enabled")

    # GUI
    find_package(Qt5Widgets REQUIRED)

    set(CMAKE_AUTORCC ON)

    FILE(GLOB  GUILIB_INCLUDES        inc/*.h )
    FILE(GLOB  GUILIB_SOURCES         src/*.cpp )
    FILE(GLOB  GUILIB_RSC             *.qrc )
    include_directories( . inc ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

    ADD_LIBRARY(guilib STATIC ${GUILIB_INCLUDES} ${GUILIB_SOURCES} ${GUILIB_RSC} )
    target_include_directories(guilib INTERFACE inc )
    target_link_libraries(guilib Qt5::Widgets maflib)
    target_compile_features(guilib PRIVATE cxx_std_17 )

ENDIF()






