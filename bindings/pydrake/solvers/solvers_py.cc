#include "drake/bindings/pydrake/solvers/solvers_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(solvers, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Solving Mathematical Programs.

If you are formulating constraints using symbolic formulas, please review the
top-level documentation for :py:mod:`pydrake.math`.
)""";

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common.value");
  py::module::import("pydrake.symbolic");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineSolversMathematicalProgram(m);
  internal::DefineSolversAugmentedLagrangian(m);
  internal::DefineSolversBranchAndBound(m);
  internal::DefineSolversMixedIntegerOptimizationUtil(m);
  internal::DefineSolversMixedIntegerRotationConstraint(m);
  internal::DefineSolversSdpaFreeFormat(m);
  internal::DefineSolversClp(m);
  internal::DefineSolversCsdp(m);
<<<<<<< HEAD
<<<<<<< HEAD
  internal::DefineSolversGurobi(m);
=======
  internal::DefineSolversDreal(m);
  internal::DefineSolversGurobi(m);
  internal::DefineSolversIbex(m);
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
  internal::DefineSolversGurobi(m);
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
  internal::DefineSolversIpopt(m);
  internal::DefineSolversMosek(m);
  internal::DefineSolversNlopt(m);
  internal::DefineSolversOsqp(m);
  internal::DefineSolversScs(m);
  internal::DefineSolversSnopt(m);
}

}  // namespace pydrake
}  // namespace drake
