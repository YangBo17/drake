load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
<<<<<<< HEAD
        repository = "bazelbuild/rules_python",  # License: Apache-2.0,
        upgrade_advice = """
        The commit (version) and sha256 here should be identical to the
        commit listed in
        drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        """,
        commit = "0.19.0",
        sha256 = "ffc7b877c95413c82bfd5482c017edcf759a6250d8b24e82f41f3c8b8d9e287e",  # noqa
=======
        repository = "bazelbuild/rules_python",  # License: Apache-2.0
        # The commit (version) and sha256 here should be identical to the
        # commit listed in
        # drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        commit = "0.10.0",
        sha256 = "56dc7569e5dd149e576941bdb67a57e19cd2a7a63cc352b62ac047732008d7e1",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        mirrors = mirrors,
    )
