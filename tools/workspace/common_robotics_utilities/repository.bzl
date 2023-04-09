load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def common_robotics_utilities_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/common_robotics_utilities",
<<<<<<< HEAD
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
        """,
        commit = "89435ebf4621f4e92d3b4c2a18aec7fa91e0cb75",
        sha256 = "7577ccf13db9111a90be946996d512d3a9693b356a7e26e4ab9a1ffb90c8ea2b",  # noqa
=======
        # When updating, ensure that any new unit tests are reflected in
        # package.BUILD.bazel and BUILD.bazel in drake. Tests may have been
        # updated in ToyotaResearchInstitute/common_robotics_utilities/test/ or
        # ToyotaResearchInstitute/common_robotics_utilities/CMakeLists.txt.ros2
<<<<<<< HEAD
        commit = "9440d72e04ef97c080b59d6d99503ccca58e5419",
        sha256 = "bbeede50eaaee21ac7869a75cb325da1f163418849dae8dba2ed8de4dfd49377",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
        commit = "d8b1a861d07e7c526e2a7dd3123d351498b53636",
        sha256 = "8d3357221aeacabc538391b1ab53bd848a8f29ddae75896912c23b7bb9c3d8d8",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
