load("@drake//tools/workspace:github.bzl", "github_archive")

def picosha2_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "okdshin/PicoSHA2",
<<<<<<< HEAD
        commit = "27fcf6979298949e8a462e16d09a0351c18fcaf2",
        sha256 = "18d82bb79c021ccf4ce58125b64691accef54237ba5194462740bacf8b39d8a9",  # noqa
=======
        commit = "1677374f23352716fc52183255a40c1b8e1d53eb",
        sha256 = "82f69e96c4ce2ba07eea2915d9300ad5d1a2303edb5a323c5a3a16bf18f484f4",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
