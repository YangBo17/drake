<<<<<<< HEAD
=======
# -*- python -*-

>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
load("@drake//tools/workspace:github.bzl", "github_archive")

def qdldl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        repository = "osqp/qdldl",
        upgrade_advice = """
        When updating this commit, see drake/tools/workspace/qdldl/README.md.
        """,
        commit = "v0.1.6",
        sha256 = "aeb1b2d76849c13e9803760a4c2e26194bf80dcc9614ae25ca6bcc404dc70d65",  # noqa
=======
        repository = "oxfordcontrol/qdldl",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v0.1.5",
        sha256 = "2868b0e61b7424174e9adef3cb87478329f8ab2075211ef28fe477f29e0e5c99",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
