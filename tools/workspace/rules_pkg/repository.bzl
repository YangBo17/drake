load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that for rules_pkg, we do NOT install its LICENSE file as part of the
# Drake install, because rules_pkg is a build-time tool only.

def rules_pkg_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_pkg",  # License: Apache-2.0
<<<<<<< HEAD
        commit = "0.8.1",
        sha256 = "99d56f7cba0854dd1db96cf245fd52157cef58808c8015e96994518d28e3c7ab",  # noqa
=======
        commit = "0.7.0",
        sha256 = "e110311d898c1ff35f39829ae3ec56e39c0ef92eb44de74418982a114f51e132",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        mirrors = mirrors,
    )
