cmake_minimum_required(VERSION 3.14)

project(orbital_physics)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_CXX_STANDARD 23 CACHE STRING "C++ standard")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CPM_USE_LOCAL_PACKAGES ON)

include(cmake/CPM.cmake)
CPMUsePackageLock(package-lock.cmake)
CPMGetPackage("Eigen")
CPMGetPackage("EnTT")
CPMGetPackage("Common")
CPMGetPackage("Boost")


if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Release")
    CPMGetPackage("TBB")
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fno-omit-frame-pointer -march=x86-64-v3 -ffast-math")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    CPMGetPackage("TBB")
    if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi /O2")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DEBUG")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -fno-omit-frame-pointer -march=x86-64-v3 -ffast-math")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g -O3")
    endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Asan")
    if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi /Od")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DEBUG")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -O3 -fsanitize=address")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g3 -O3 -fsanitize=address")
    endif()
endif()

if(WIN32)
    target_compile_definitions(orbital_physics PRIVATE NOGDI NOUSER WIN32_LEAN_AND_MEAN)

    set(PROJECT_DIR ${CMAKE_SOURCE_DIR})
    set(BUILD_DIR ${CMAKE_BINARY_DIR})
    set(PHYSICS_DIR "${BUILD_DIR}/Physics/${CMAKE_BUILD_TYPE}")

    set(TARGET_DIR "${PROJECT_DIR}")
endif()

add_library(orbital_physics SHARED
    src/PhysicsAPI.cpp
    src/NewtonianPhysics.cpp
    src/forces/Gravity.cpp
    src/integration/Verlet.cpp
)

target_link_libraries(orbital_physics PRIVATE
    Eigen3::Eigen
    EnTT::EnTT
    Common
    Boost::iterator
    Boost::align
    $<$<CONFIG:Release>:TBB::tbb>
    $<$<CONFIG:Debug>:TBB::tbb>
)

target_include_directories(orbital_physics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Common_SOURCE_DIR}/include
)
