cmake_minimum_required(VERSION 3.16)

project(ABS LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)

add_library(ABS
    src/abs.cpp
    src/velocity_ekf.cpp
)

target_include_directories(ABS
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/TinyEKF/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(BUILD_TESTING)
    add_executable(abs_behavior_test
        tests/abs_behavior_test.cpp
    )

    target_link_libraries(abs_behavior_test
        PRIVATE
            ABS
    )

    add_test(NAME abs_behavior_test COMMAND abs_behavior_test)
endif()
