cmake_minimum_required(VERSION 3.0)

PROJECT(maflib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(inc)
FILE(GLOB_RECURSE  MAF_INCLUDES        inc/*.h)
FILE(GLOB_RECURSE  MAF_SOURCES         src/*.cpp)

ADD_LIBRARY(maflib SHARED ${MAF_INCLUDES} ${MAF_SOURCES} )
target_include_directories(maflib INTERFACE inc)
target_link_libraries(maflib Eigen3::Eigen )
target_compile_features(maflib PRIVATE cxx_std_17 )

option(TESTMAF  "TEST" ON)
IF(${TESTMAF})
    MESSAGE(STATUS "MAF tests activated")
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    FILE(GLOB_RECURSE  MAF_TESTS_INC       test/*.h)
    FILE(GLOB_RECURSE  MAF_TESTS_SRC       test/*.cpp)

    add_executable(runMafLibTests ${MAF_TESTS_INC} ${MAF_TESTS_SRC})
    target_link_libraries(runMafLibTests ${GTEST_LIBRARIES} pthread maflib )
    target_compile_features(runMafLibTests PRIVATE cxx_std_17 )
ENDIF()

