load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
<<<<<<< HEAD
        commit = "61c02fb47fa90ac1a5daa83e22d8f39bb1348aaf",
        sha256 = "58a8485ec1e6001f3987f5a6d7e3f8ee38af898aa5f93a07e1c4f5025ef42de7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
=======
        commit = "15afd9aa83a9f25964add3d45f57aed7c2484142",
        sha256 = "b0451c92c453ca7844a26516eb2fb27beebd86e2afb31c4a2b5e2487d2fd1d6c",  # noqa
        build_file = ":package.BUILD.bazel",
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        mirrors = mirrors,
    )
