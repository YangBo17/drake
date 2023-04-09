# Note: use setuptools.glob rather than the built-in glob; see
# https://bugs.python.org/issue37578.
import os
import setuptools
from setuptools import setup, find_packages, glob

DRAKE_VERSION = os.environ.get('DRAKE_VERSION', '0.0.0')

# Required python packages that will be pip installed along with pydrake
# TODO Can we remove any of these?
python_required = [
    'matplotlib',
    'meshcat',
    'numpy',
    'pydot',
    'PyYAML',
]

if os.uname()[0].lower() == 'linux':
    # This is intended to help force a binary, rather than platform-agnostic,
    # wheel, but only works on Ubuntu; clang is not happy about being asked to
    # make a library with no sources.
    ext_modules = [
        setuptools.Extension(name='drake',
                             sources=[]),
    ]
else:
    ext_modules = []


def find_data_files(*patterns):
    result = []
    for pattern in patterns:
        result += [f'../{f}' for f in glob.iglob(pattern, recursive=True)]
    return result


def _actually_find_packages():
    """Work around broken(?!) setuptools."""
    result = find_packages()
<<<<<<< HEAD
    result.extend([
<<<<<<< HEAD
        'pydrake.examples',
        'pydrake.geometry',
        'pydrake.manipulation',
        'pydrake.solvers',
        'pydrake.visualization',
=======
        "pydrake.examples",
        "pydrake.geometry",
        "pydrake.solvers",
        "pydrake.visualization",
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
    ])
    print(f'Using packages={result}')
    return result


# Generate a source file we can use to produce an extension library (which we
# do to force the wheel to not be platform-agnostic. We need this because
# trying to build an extension module with no sources is not reliable.
with open('dummy.c', 'wt') as f:
    f.write('void not_used() {}')


=======
    result.extend(["pydrake.examples", "pydrake.solvers"])
    print(f"Using packages={result}")
    return result


>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
setup(name='drake',
      version=DRAKE_VERSION,
      description='Model-based design and verification for robotics',
      long_description='''
Drake ("dragon" in Middle English) is a toolbox started by the Robot Locomotion
Group at the MIT Computer Science and Artificial Intelligence Lab (CSAIL).
The development team has now grown significantly, with core development led by
the Toyota Research Institute.
It is a collection of tools for analyzing the dynamics of our robots and
building control systems for them, with a heavy emphasis on optimization-based
design/analysis.

See https://drake.mit.edu/pip.html for installation instructions and caveats.
'''.strip(),
      url='https://drake.mit.edu',
      author='Drake Development Team',
      author_email='drake-users@mit.edu',
      classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',
        'License :: Other/Proprietary License',
        'Operating System :: MacOS',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: Implementation :: CPython',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries :: Python Modules'],
<<<<<<< HEAD
      license='Various',
      platforms=['linux_x86_64', 'macosx_x86_64', 'macosx_arm64'],
<<<<<<< HEAD
=======
      distclass=BinaryDistribution,
      # TODO Check this: do we need to add third-party licenses?
      license='BSD 3-Clause License',
      platforms=['linux_x86_64', 'macosx_x86_64'],
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
      packages=_actually_find_packages(),
      # Add in any packaged data.
      include_package_data=True,
      package_data={
          '': find_data_files(
              'pydrake/py.typed',
              'pydrake/**/*.pyi',
              'pydrake/**/*.so',
              'pydrake/lib/**',
              'pydrake/doc/**',
              'pydrake/share/**',
              'pydrake/INSTALLATION',
          )
      },
      python_requires='>=3.8',
      install_requires=python_required,
<<<<<<< HEAD
      # Ensure the wheel is not platform-agnostic.
      ext_modules=[
        setuptools.Extension(name='drake',
                             sources=['dummy.c']),
      ],
=======
      ext_modules=ext_modules,
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
      zip_safe=False
      )
