load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
<<<<<<< HEAD
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "ea474eb1e7b595b45145c8104fc9684d49f15231",
        sha256 = "609988dcb6ca3090121ae0b0a149e0c1fab9656a8f2e867786e666264a1d42ca",  # noqa
=======
        # Updating this commit requires local testing; see
        # drake/tools/workspace/meshcat/README.md for details.
        commit = "65781fcb064db536b99a66fe9fcf5bf0b6d1f790",
        sha256 = "d55918a9d14b1f92b331e5d64df7be1572c670afcc4dd5f46372c186708b9e80",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
