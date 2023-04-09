load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
<<<<<<< HEAD
        commit = "4ae8771a314abe8b0e85cee3be3ead30451c63e7",
        sha256 = "43ed382a005d81de51f4d17358b66e56c008152440bd0a59cbc4bd8e69a54214",  # noqa
        patches = [
            ":patches/disable_int128_on_clang.patch",
=======
        commit = "f4988f5bd4176345aad2a525e24d5fd11b3c97ea",
        sha256 = "56c94a5e2d413110f7299ad388f4c816b9858e6e996042799754279d46be77a9",  # noqa
        patches = [
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
            ":patches/hidden_visibility.patch",
            ":patches/inline_namespace.patch",
        ],
        patch_cmds = [
            # Force linkstatic = 1 everywhere. First, remove the few existing
            # uses so that we don't get "duplicate kwarg" errors. Then, add it
            # anywhere that linkopts already appears.
            "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts =|' absl/*/BUILD.bazel",  # noqa
        ],
        mirrors = mirrors,
    )
