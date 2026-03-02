cmake_minimum_required(VERSION 3.14)

file(GLOB_RECURSE src "*.cpp")
file(GLOB_RECURSE common_src "${CMAKE_SOURCE_DIR}/common/src/*.cpp")

add_library(orbital_physics SHARED ${src} ${common_src})

target_link_libraries(orbital_physics PRIVATE Eigen3::Eigen)

target_include_directories(orbital_physics PRIVATE
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/
)

if(WIN32)
    target_compile_definitions(orbital_physics PRIVATE NOGDI NOUSER WIN32_LEAN_AND_MEAN)

    set(PROJECT_DIR ${CMAKE_SOURCE_DIR})
    set(BUILD_DIR ${CMAKE_BINARY_DIR})
    set(PHYSICS_DIR "${BUILD_DIR}/physics/${CMAKE_BUILD_TYPE}")

    set(TARGET_DIR "${PROJECT_DIR}")
endif()
