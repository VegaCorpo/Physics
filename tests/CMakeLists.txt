add_executable(orbital_physics_tests
    main.cpp
    forces/GravityTest.cpp
    integration/VerletTest.cpp
    physics/NewtonianPhysicsTest.cpp
    events/GravityEventTest.cpp
)

target_link_libraries(orbital_physics_tests PRIVATE
    orbital_physics
    GTest::gtest
    GTest::gmock
    EnTT::EnTT
    Eigen3::Eigen
)

target_include_directories(orbital_physics_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${Common_SOURCE_DIR}/include
)

include(GoogleTest)
gtest_discover_tests(orbital_physics_tests)