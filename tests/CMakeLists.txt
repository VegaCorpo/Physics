cmake_minimum_required(VERSION 3.14)

# Standalone QA project for the Physics module.
#
# It is intentionally NOT wired into ../CMakeLists.txt: it has its own
# dependencies and never links the physics library. The runner dlopen()s a
# prebuilt liborbital_physics.so and talks to it only through the public
# common::IPhysicsEngine interface, so the same runner can drive any commit
# of the module (and any other module implementing the interface).
#
# PHYSICS_SOURCE_DIR points at the checkout of the module whose library will be
# loaded. Its package-lock.cmake decides which Common version is used, so the
# runner is always ABI-compatible with the library it tests.

project(orbital_physics_qa LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23 CACHE STRING "C++ standard")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CPM_USE_LOCAL_PACKAGES ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(PHYSICS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH
    "Checkout of the physics module whose package-lock.cmake provides Common")

include(cmake/CPM.cmake)

if(NOT EXISTS "${PHYSICS_SOURCE_DIR}/package-lock.cmake")
    message(FATAL_ERROR "PHYSICS_SOURCE_DIR=${PHYSICS_SOURCE_DIR} has no package-lock.cmake")
endif()
include("${PHYSICS_SOURCE_DIR}/package-lock.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/package-lock.cmake")

CPMGetPackage("Common")
CPMGetPackage("nlohmann_json")

add_executable(physics_qa_runner
    runner/main.cpp
)

target_compile_options(physics_qa_runner PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O2 -Wall -Wextra>
)

target_include_directories(physics_qa_runner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/runner
)

target_link_libraries(physics_qa_runner PRIVATE
    Common
    nlohmann_json::nlohmann_json
    ${CMAKE_DL_LIBS}
)
