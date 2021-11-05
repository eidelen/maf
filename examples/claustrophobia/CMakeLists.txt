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
    target_link_libraries(claustrophobia Qt5::Widgets pthread maflib )

    option(TESTCL  "Test claustrophobia" OFF)
    IF(${TESTCL})
        MESSAGE(STATUS "Tests claustrophobia activated")
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIRS})

        FILE(GLOB_RECURSE  CL_TESTS_INC       test/*.h)
        FILE(GLOB_RECURSE  CL_TESTS_SRC       test/*.cpp)

        add_executable(runCLTests ${CL_TESTS_INC} ${CL_TESTS_SRC} )
        target_link_libraries(runCLTests ${GTEST_LIBRARIES} Qt5::Widgets pthread maflib )
        target_compile_features(runCLTests PRIVATE cxx_std_17 )
    ENDIF()

ENDIF()






