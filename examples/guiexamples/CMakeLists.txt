cmake_minimum_required(VERSION 3.0)

PROJECT(guiexamples)

option(GUIEXENABLED  "GUI Examples" OFF)
IF(${GUIEXENABLED})
    MESSAGE(STATUS "GUI examples activated")

    # GUI
    find_package(Qt5Widgets REQUIRED)

    SET(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    include_directories( . gui ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

    FILE(GLOB  GUIEX_INCLUDES        *.h gui/*.h)
    FILE(GLOB  GUIEX_SOURCES         *.cpp gui/*.cpp)

    add_executable(guiexamples ${GUIEX_INCLUDES} ${GUIEX_SOURCES} )
    target_link_libraries(guiexamples Qt5::Widgets maflib guilib)
    target_compile_features(guiexamples PRIVATE cxx_std_17 )

ENDIF()






