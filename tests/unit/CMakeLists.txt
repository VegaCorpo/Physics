cmake_minimum_required(VERSION 3.14)

# White-box unit tests for the collision module.
#
# Unlike ../CMakeLists.txt (the black-box runner), this project compiles the
# module's Octree and Collider sources directly: common::IPhysicsEngine does not
# expose the collision pairs, so the only way to verify them is to call
# physics::Collider::checkCollisions() and compare its output with an O(n^2)
# brute-force reference.
#
# PHYSICS_SOURCE_DIR points at the checkout of the module under test. Its
# package-lock.cmake decides which Common and Boost versions are used, exactly
# like the runner project.

project(orbital_physics_unit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23 CACHE STRING "C++ standard")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CPM_USE_LOCAL_PACKAGES ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(PHYSICS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH
    "Checkout of the physics module whose sources are compiled into the tests")

include(../cmake/CPM.cmake)

if(NOT EXISTS "${PHYSICS_SOURCE_DIR}/package-lock.cmake")
    message(FATAL_ERROR "PHYSICS_SOURCE_DIR=${PHYSICS_SOURCE_DIR} has no package-lock.cmake")
endif()
include("${PHYSICS_SOURCE_DIR}/package-lock.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/package-lock.cmake")

CPMGetPackage("Common")
CPMGetPackage("Boost")
CPMGetPackage("GTest")

enable_testing()

add_executable(physics_unit_tests
    collider_test.cpp
    octree_test.cpp
    ${PHYSICS_SOURCE_DIR}/src/spatial/Octree.cpp
    ${PHYSICS_SOURCE_DIR}/src/collisions/collider.cpp
)

target_compile_options(physics_unit_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
)

target_include_directories(physics_unit_tests PRIVATE
    ${PHYSICS_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(physics_unit_tests PRIVATE
    Common
    Boost::align
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(physics_unit_tests)
