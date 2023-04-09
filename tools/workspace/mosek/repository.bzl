"""
Downloads and unpacks a MOSEK™ archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")  # noqa
        mosek_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:mosek"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:execute.bzl", "which")
load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
<<<<<<< HEAD
    # When these values are updated:
    # - tools/dynamic_analysis/tsan.supp may also need updating
    # - LICENSE.third_party may also need updating to match
    #     https://docs.mosek.com/latest/licensing/license-agreement-info.html
    mosek_major_version = 10
    mosek_minor_version = 0
    mosek_patch_version = 18

    os_result = determine_os(repository_ctx)
    if os_result.is_macos or os_result.is_macos_wheel:
        if os_result.macos_arch_result == "arm64":
            mosek_platform = "osxaarch64"
            sha256 = "99518b88c3bfc27edc92775eada349f7dd232c7bf7aa1cb7a8620603e05a6a6d"  # noqa
        else:
            mosek_platform = "osx64x86"
            sha256 = "e3de2b99e5ab27a7c37356a7fe88f0a42c53ec04aeaba70abe8b5971fbcfc150"  # noqa
    elif os_result.is_ubuntu or os_result.is_manylinux:
        mosek_platform = "linux64x86"
        sha256 = "f778f6e5560cdb8a3b5001cb51f40ccba9b3ef73da09406dcd3c1a870433eb34"  # noqa
=======
    # When these values are updated, tools/dynamic_analysis/tsan.supp may also
    # need updating.
    mosek_major_version = 9
    mosek_minor_version = 3
    mosek_patch_version = 20

    if repository_ctx.os.name == "mac os x":
        mosek_platform = "osx64x86"
        sha256 = "e804225fdc48933d753645e6e4afe415aabbabb32233dd376d0dd6bf985756ef"  # noqa
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "2fa2e1f742a31d7a7249ae083748f377dc68e378eb5ba18279647a433dc2e595"  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    else:
        fail(
            "Operating system is NOT supported",
            attr = repository_ctx.os.name,
        )

    # TODO(jwnimmer-tri) Port to use mirrors.bzl.
    template = "https://download.mosek.com/stable/{}.{}.{}/mosektools{}.tar.bz2"  # noqa
    url = template.format(
        mosek_major_version,
        mosek_minor_version,
        mosek_patch_version,
        mosek_platform,
    )
    root_path = repository_ctx.path("")
    strip_prefix = "mosek/{}.{}".format(
        mosek_major_version,
        mosek_minor_version,
    )

    repository_ctx.download_and_extract(
        url,
        root_path,
        sha256 = sha256,
        stripPrefix = strip_prefix,
    )

    platform_prefix = "tools/platform/{}".format(mosek_platform)

    if repository_ctx.os.name == "mac os x":
        install_name_tool = which(repository_ctx, "install_name_tool")

<<<<<<< HEAD
=======
        # Note that libmosek64.dylib is (erroneously) a copy of
        # libmosek64.9.3.dylib instead of a symlink. Otherwise, the list of
        # files should include the following in place of bin/libmosek64.dylib:
        #
        # "bin/libmosek64.{}.{}.dylib".format(mosek_major_version,
        #                                     mosek_minor_version)
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        files = [
            "bin/libtbb.12.dylib",
            "bin/libtbb.12.5.dylib",
            "bin/libmosek64.{}.{}.dylib".format(
                mosek_major_version,
                mosek_minor_version,
            ),
        ]

        for file in files:
            file_path = repository_ctx.path(
                "{}/{}".format(platform_prefix, file),
            )

            result = repository_ctx.execute([
                install_name_tool,
                "-id",
                file_path,
                file_path,
            ])

            if result.return_code != 0:
                fail(
                    "Could NOT change shared library identification name",
                    attr = result.stderr,
                )

        srcs = []

        bin_path = repository_ctx.path("{}/bin".format(platform_prefix))

        linkopts = [
            "-L{}".format(bin_path),
            "-lmosek64",
        ]
    else:
        files = [
            # We use the the MOSEK™ copy of libtbb. The version of libtbb
            # available in Ubuntu is too old.
            "bin/libtbb.so.12",
            "bin/libtbb.so.12.6",
            "bin/libmosek64.so.{}.{}".format(
                mosek_major_version,
                mosek_minor_version,
            ),
        ]

        linkopts = ["-pthread"]
        srcs = ["{}/{}".format(platform_prefix, file) for file in files]

    hdrs = ["{}/h/mosek.h".format(platform_prefix)]
    includes = ["{}/h".format(platform_prefix)]
    files = ["{}/{}".format(platform_prefix, file) for file in files]
    libraries_strip_prefix = ["{}/bin".format(platform_prefix)]

    file_content = """# DO NOT EDIT: generated by mosek_repository()

load("@drake//tools/install:install.bzl", "install", "install_files")

licenses([
    "by_exception_only",  # MOSEK
    "notice",  # fplib AND Zlib
])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mosek",
    srcs = {},
    hdrs = {},
    includes = {},
    linkopts = {},
)

install_files(
    name = "install_libraries",
    dest = "lib",
    files = {},
    strip_prefix = {},
    visibility = ["//visibility:private"],
)

install(
    name = "install",
    docs = [
        "mosek-eula.pdf",
        "@drake//tools/workspace/mosek:drake_mosek_redistribution.txt",
        "@drake//tools/workspace/mosek:LICENSE.third_party",
    ],
    doc_strip_prefix = ["tools/workspace/mosek"],
    allowed_externals = ["@drake//:.bazelproject"],
    deps = [":install_libraries"],
)
    """.format(srcs, hdrs, includes, linkopts, files, libraries_strip_prefix)

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

mosek_repository = repository_rule(implementation = _impl)
