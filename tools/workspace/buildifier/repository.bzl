"""
Downloads a precompiled version of buildifier and makes it available to the
WORKSPACE.

Example:
    WORKSPACE:
        load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
        load("@drake//tools/workspace/buildifier:repository.bzl", "buildifier_repository")  # noqa
        buildifier_repository(name = "foo", mirrors = DEFAULT_MIRRORS)

    BUILD:
        sh_binary(
            name = "foobar",
            srcs = ["bar.sh"],
            data = ["@foo//:buildifier"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")
load(
    "@drake//tools/workspace:metadata.bzl",
    "generate_repository_metadata",
)

def _impl(repository_ctx):
    # Enumerate the possible binaries.  Note that the buildifier binaries are
    # fully statically linked, so the particular distribution doesn't matter,
    # only the kernel.
<<<<<<< HEAD
<<<<<<< HEAD
    version = "6.0.1"
=======
    version = "5.1.0"
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
    version = "6.0.1"
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
    darwin_urls = [
        x.format(version = version, filename = "buildifier-darwin-amd64")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
<<<<<<< HEAD
<<<<<<< HEAD
    darwin_sha256 = "f134ec0a16b5da6f006a4e65ec1ec6044efd3b149a85fae913db57d1be0fbc70"  # noqa
=======
    darwin_sha256 = "c9378d9f4293fc38ec54a08fbc74e7a9d28914dae6891334401e59f38f6e65dc"  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
    darwin_sha256 = "f134ec0a16b5da6f006a4e65ec1ec6044efd3b149a85fae913db57d1be0fbc70"  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
    linux_urls = [
        x.format(version = version, filename = "buildifier-linux-amd64")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
<<<<<<< HEAD
<<<<<<< HEAD
    linux_sha256 = "0e0f00a5053d697e1644d1b660dc1fb81b55838519751dd2f09eb1bd84c6b51c"  # noqa
=======
    linux_sha256 = "52bf6b102cb4f88464e197caac06d69793fa2b05f5ad50a7e7bf6fbd656648a3"  # noqa
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
    linux_sha256 = "0e0f00a5053d697e1644d1b660dc1fb81b55838519751dd2f09eb1bd84c6b51c"  # noqa
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f

    # Choose which binary to use.
    os_result = determine_os(repository_ctx)
    if os_result.is_macos:
        urls = darwin_urls
        sha256 = darwin_sha256
    elif os_result.is_ubuntu or os_result.is_manylinux:
        urls = linux_urls
        sha256 = linux_sha256
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    # Fetch the binary from mirrors.
    output = repository_ctx.path("buildifier")
    repository_ctx.download(urls, output, sha256, executable = True)

    # Add the BUILD file.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/buildifier:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Create a summary file for Drake maintainers.  We need to list all
    # possible binaries so Drake's mirroring scripts will fetch everything.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "manual",
        version = version,
        downloads = [
            {
                "urls": darwin_urls,
                "sha256": darwin_sha256,
            },
            {
                "urls": linux_urls,
                "sha256": linux_sha256,
            },
        ],
    )

buildifier_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
