"""
On Ubuntu, pkg-config is used to locate the msgpack headers. On macOS,
no pkg-config file is installed, but the msgpack headers are always
located at ${homebrew_prefix}/opt/msgpack-cxx/include.
"""

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)
load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_ubuntu:
        error = setup_pkg_config_repository(repo_ctx).error
        if error != None:
            fail(error)
    else:
        prefix = "{}/opt/msgpack-cxx/".format(os_result.homebrew_prefix)
        repo_ctx.symlink("{}/include".format(prefix), "msgpack")

        hdrs_patterns = [
            "msgpack/**/*.h",
            "msgpack/**/*.hpp",
        ]

        deprecation = repo_ctx.attr.extra_deprecation

        file_content = """# DO NOT EDIT: generated by msgpack_repository()

licenses(["notice"])  # Boost-1.0

cc_library(
    name = "msgpack",
    hdrs = glob({}),
    includes = ["msgpack"],
    defines = ["MSGPACK_NO_BOOST"],
    visibility = ["//visibility:public"],
<<<<<<< HEAD
    deprecation = {},
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
)
    """.format(hdrs_patterns, repr(deprecation))

        repo_ctx.file(
            "BUILD.bazel",
            content = file_content,
            executable = False,
        )

msgpack_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "msgpack"),
        "licenses": attr.string_list(default = ["notice"]),  # Boost-1.0
        "extra_deprecation": attr.string(default = "DRAKE DEPRECATED: The @msgpack external is deprecated. The deprecated code will be removed from Drake on or after 2023-05-01."),  # noqa
    },
    local = True,
    configure = True,
    implementation = _impl,
)
