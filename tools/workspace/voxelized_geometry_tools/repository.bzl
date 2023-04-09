load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/voxelized_geometry_tools",
<<<<<<< HEAD
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake.
        """,
        commit = "83f9d52218fcd31a91b3cb493bedbad53fdf2bb3",
        sha256 = "2a04e90bb6ffcf34b502422b808a90c729ae495c525594957c3cdc9db4dce04c",  # noqa
=======
        # When updating, ensure that any new unit tests are reflected in
        # package.BUILD.bazel and BUILD.bazel in drake.
<<<<<<< HEAD
        commit = "c940ec07ecb4f109712d2be071798a00646823ca",
        sha256 = "b7718ca30b46c6d4420b3ed2f7890bce998bee659d056954c9051a9cba202404",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
        commit = "83f9d52218fcd31a91b3cb493bedbad53fdf2bb3",
        sha256 = "2a04e90bb6ffcf34b502422b808a90c729ae495c525594957c3cdc9db4dce04c",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
