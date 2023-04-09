load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "petsc/petsc",
<<<<<<< HEAD
<<<<<<< HEAD
        commit = "v3.18.5",
        sha256 = "2c96efe5c7ad1dd9f0e4138e8c1b5622d5a3de48be0971968cd8318f9838657c",  # noqa
=======
        commit = "v3.17.3",
        sha256 = "7ff5bc5e58057761f94004e79e5da73d3cd308699a9f244fe4406abdad7a521a",  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
        commit = "v3.18.4",
        sha256 = "8da1da9588bc97cd162f76ef92ceeca4b188ba3b8842aa2f7a59043015021785",  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # Cherry-picked from upstream (to be removed once
            # https://gitlab.com/petsc/petsc/-/merge_requests/5228/commits
            # is included in the release).
            ":patches/baij.patch",
            # Patch to fix dangerous global state in PETSc.
            ":patches/destroy.patch",
            ":patches/dlregispetsc.patch",
            ":patches/inherit.patch",
            ":patches/matrix.patch",
            ":patches/mpi.patch",
            ":patches/petscimpl.patch",
<<<<<<< HEAD
            ":patches/petsc_creationidx_keyval.patch",
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
            ":patches/pname.patch",
            ":patches/remove_packages.patch",
            ":patches/tagm.patch",
        ],
    )
