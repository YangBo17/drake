"""Shim module that provides vestigial names for pydrake.examples.
Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.examples directly.
<<<<<<< HEAD
"""

from pydrake.common.deprecation import _warn_deprecated

=======
This module will be deprecated at some point in the future.
"""

>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
from pydrake.examples import (
    CreateClutterClearingYcbObjectList,
    CreateManipulationClassYcbObjectList,
    IiwaCollisionModel,
    ManipulationStation,
    ManipulationStationHardwareInterface,
    SchunkCollisionModel,
)
<<<<<<< HEAD

_warn_deprecated(
    "Please import from the pydrake.examples module directly, instead of the "
    f"deprecated {__name__} submodule.",
    date="2023-05-01", stacklevel=3)
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
