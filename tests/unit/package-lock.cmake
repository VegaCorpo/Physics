# Dependencies that belong to the unit tests only. Common and Boost come from
# the package-lock.cmake of the Physics checkout under test.
# With CPM_USE_LOCAL_PACKAGES a system GTest (find_package) is preferred.
CPMDeclarePackage(GTest
    NAME GTest
    GIT_TAG v1.17.0
    GITHUB_REPOSITORY google/googletest
    SYSTEM YES
    EXCLUDE_FROM_ALL YES
    OPTIONS "INSTALL_GTEST OFF" "gtest_force_shared_crt ON"
)
