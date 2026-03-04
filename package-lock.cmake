# Eigen
CPMDeclarePackage(Eigen
        GIT_TAG 5.0.1
        GITLAB_REPOSITORY libeigen/eigen
        SYSTEM YES
        EXCLUDE_FROM_ALL YES
)

#entt
CPMDeclarePackage(EnTT
    GIT_TAG v3.16.0
    GITHUB_REPOSITORY skypjack/entt
    SYSTEM YES
    EXCLUDE_FROM_ALL YES
)

# Common
CPMDeclarePackage(Common
        GIT_TAG main
        GITHUB_REPOSITORY VegaCorpo/Common
        SYSTEM YES
        EXCLUDE_FROM_ALL YES
        DOWNLOAD_ONLY YES
)