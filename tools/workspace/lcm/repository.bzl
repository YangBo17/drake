load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
<<<<<<< HEAD
        upgrade_advice = """
        When upgrading this commit, check if the LCM maintainers have tagged
        a new version number; if so, then update the version numbers within
        the two lcm-*.cmake files in this directory to match.
        """,
        commit = "1aecca45e6a05d719da8e566533e45740d1fd88c",
        sha256 = "78ef84ccabf78ebb33a182393bb539f38fedb576e2c1cb244bf134470bd702e3",  # noqa
=======
        # When upgrading this commit, check if the LCM maintainers have tagged
        # a new version number; if so, then update the version numbers within
        # the two lcm-*.cmake files in this directory to match.
        commit = "91ce7a2ae46ad05f8a232f5fe32a06cccbead1c2",
        sha256 = "8ea0076d2f2158fc750fec697b68c6903a9d70ccbe4e3f24240415a13445381f",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
