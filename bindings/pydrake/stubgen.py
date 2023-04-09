"""Command-line tool to generate Drake's Python Interface files (.pyi)."""

import sys
<<<<<<< HEAD
<<<<<<< HEAD
import warnings

from mypy import stubgen

from pydrake.common.deprecation import DrakeDeprecationWarning

if __name__ == "__main__":
    warnings.simplefilter('ignore', DrakeDeprecationWarning)

    # Mypy can time out if importing takes an inordinate length of time.
    # Try to avoid this by importing ourselves up front when the import
    # isn't being run under a timeout.
    import pydrake.all

=======
=======
import warnings
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f

from mypy import stubgen

from pydrake.common.deprecation import DrakeDeprecationWarning

if __name__ == "__main__":
<<<<<<< HEAD
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
    warnings.simplefilter('ignore', DrakeDeprecationWarning)

    # Mypy can time out if importing takes an inordinate length of time.
    # Try to avoid this by importing ourselves up front when the import
    # isn't being run under a timeout.
    import pydrake.all

>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
    sys.exit(stubgen.main())
