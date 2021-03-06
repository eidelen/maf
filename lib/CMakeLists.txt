cmake_minimum_required(VERSION 3.0)

PROJECT(maflib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED NO_MODULE)

SET(MAF_INC_FOLDERS inc inc/agents inc/objectives)

INCLUDE_DIRECTORIES(${MAF_INC_FOLDERS})
FILE(GLOB  MAF_INCLUDES        inc/*.h inc/agents/*.h inc/objectives/*.h)
FILE(GLOB  MAF_SOURCES         src/*.cpp src/agents/*.cpp src/objectives/*.cpp)

ADD_LIBRARY(maflib STATIC ${MAF_INCLUDES} ${MAF_SOURCES} )
target_include_directories(maflib INTERFACE ${MAF_INC_FOLDERS} )
target_link_libraries(maflib Eigen3::Eigen )
target_compile_features(maflib PRIVATE cxx_std_17 )

option(TESTMAF  "TEST" ON)
IF(${TESTMAF})
    MESSAGE(STATUS "MAF tests activated")

    # Download and build gtest
    include(FetchContent)
    FetchContent_Declare(
        googletest
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)


    FILE(GLOB_RECURSE  MAF_TESTS_INC       test/*.h)
    FILE(GLOB_RECURSE  MAF_TESTS_SRC       test/*.cpp)

    add_executable(runMafLibTests ${MAF_TESTS_INC} ${MAF_TESTS_SRC})
    target_link_libraries(runMafLibTests maflib gtest_main  )
    target_compile_features(runMafLibTests PRIVATE cxx_std_17 )
ENDIF()

