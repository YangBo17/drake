load("@drake//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
<<<<<<< HEAD
        commit = "1.4.1",
        sha256 = "060426b186670beede4104095324a72bd7494d8b4e785bf0d84a612978285908",  # noqa
=======
        commit = "1.4.0",
        sha256 = "4dd05f44200db3b78f72f56ebd8b102d5bcdc17c0299955d4eb20c38c6f07cd7",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        mirrors = mirrors,
    )
