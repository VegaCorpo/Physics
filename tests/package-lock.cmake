# Dependencies that belong to the QA tooling only.
# Common is NOT declared here: it is resolved from the package-lock.cmake of
# the Physics checkout under test so the runner shares the exact ABI of the
# library it loads.
CPMDeclarePackage(nlohmann_json
    GIT_TAG v3.12.0
    GITHUB_REPOSITORY nlohmann/json
    SYSTEM YES
    EXCLUDE_FROM_ALL YES
)
