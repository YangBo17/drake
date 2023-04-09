load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
<<<<<<< HEAD
<<<<<<< HEAD
        commit = "54684096e4ab1fcff9e7571888489e48d018c7fb",
        sha256 = "18668e5c9fa22404b796d661732a44f7adbeb1300542120097531eef17fd4917",  # noqa
=======
        commit = "a1e8bad32e1ccd26a7936c5354ecf856aec2cf59",
        sha256 = "1b23bb6feb24cc3faac8a31c8470d268a679be89f7761d6fb488897b2606ca33",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
        commit = "ed48fa93268ef2c3e930370fd2fa760de30aee18",
        sha256 = "f34aca354e538d0d1a277b2b13d468b650ab3ca5efd9b02cf50c66fa1a213f8b",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # We select tinyobjloader's floating-point typedef using a patch
            # file instead of `defines = []` because tinyobjloader is a private
            # dependency of Drake and we don't want the definition to leak into
            # all target that consume Drake.
            ":double_precision.patch",
            # We replace tinyobjloader's implementation of float parsing with a
            # faster call to strtod_l.
            ":faster_float_parsing.patch",
        ],
    )
