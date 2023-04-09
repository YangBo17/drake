<<<<<<< HEAD
"""Validates Drake's Python Interface files (*.pyi)."""

import os.path
import unittest

from bazel_tools.tools.python.runfiles import runfiles
import mypy.fastparse


class TestStubgen(unittest.TestCase):

    def _rlocation(self, resource_path):
        manifest = runfiles.Create()
        result = manifest.Rlocation(resource_path)
        assert os.path.exists(result), result
        return result

    def _expected_pyi_files(self):
        """Returns (stubs_dir, paths) -- the base directory for *.pyi files and
        the list of relative paths to all of them.
        """
        this_bazel_package = "drake/bindings/pydrake"
        stubs_dir = self._rlocation(this_bazel_package)
        # N.B. the `_pyi_files` data comes from a custom BUILD.bazel genrule.
        with open(self._rlocation(f"{this_bazel_package}/_pyi_files")) as f:
            paths = f.read().splitlines()
        return stubs_dir, paths

    def test_valid_syntax(self):
        """Checks that all *.pyi files can be parsed by Mypy.
        """
        stubs_dir, paths = self._expected_pyi_files()
        self.assertGreater(len(paths), 0)
        for path in paths:
            with self.subTest(pyi=path):
                self._check_one_valid_syntax(stubs_dir, path)

    def _check_one_valid_syntax(self, stubs_dir, path):
        """Helper for test_valid_syntax.
        """
        with open(f"{stubs_dir}/{path}", "r") as f:
            source = f.read()
        # Check that the parser doesn't raise any exceptions. For compilation
        # errors in particular, we'll improve the error message to show the
        # problematic line.
        failure_message = None
        try:
            mypy.fastparse.parse(source=source, fnam=path, module=None)
        except mypy.errors.CompileError as e:
            message = e.messages[0]
            line_num = int(message.split(":", maxsplit=2)[1])
            line_str = source.splitlines()[line_num - 1]
            failure_message = f"{message}:\n{line_str}"
        if failure_message is not None:
            self.fail(failure_message)
=======
"""Tests generation of Drake's Python Interface files (.pyi)."""

import os
import unittest

from mypy import stubgen

# Mypy can time out if importing takes an inordinate length of time. Try to
# avoid this by importing ourselves up front when the import isn't being run
# under a timeout.
import pydrake.all


class TestStubgen(unittest.TestCase):
    # TODO(mwoehlke-kitware): test the already-generated files instead.
    def test_generation(self):
        """Ensure that stubgen runs and generates output.

        For now, this is more or less just a smoke test, with a very cursory
        check that the output is 'reasonable'.
        """
        output_dir = os.environ['TEST_TMPDIR']
        args = ['--package', 'pydrake', '--output', output_dir]

        # Generate stubs.
        result = stubgen.main(args)
        self.assertTrue(result is None or result == 0)

        # Find some of the expected output and look for an expected function.
        expected = os.path.join(output_dir, 'pydrake', '__init__.pyi')
        found_expected_decl = False
        for line in open(expected, 'r'):
            if line.startswith('def getDrakePath():'):
                found_expected_decl = True
                break
        self.assertTrue(found_expected_decl)
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
