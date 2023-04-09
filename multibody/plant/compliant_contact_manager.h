#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/contact_pair_kinematics.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
// Forward declaration.
struct SapSolverParameters;
}  // namespace internal
}  // namespace contact_solvers
namespace internal {

// Forward declaration.
template <typename>
class SapDriver;
template <typename>
class TamsiDriver;

// To compute accelerations due to external forces (in particular non-contact
// forces), we pack forces, ABA cache and accelerations into a single struct
// to confine memory allocations into a single cache entry.
template <typename T>
struct AccelerationsDueToExternalForcesCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationsDueToExternalForcesCache)
  explicit AccelerationsDueToExternalForcesCache(
      const MultibodyTreeTopology& topology);
  MultibodyForces<T> forces;  // The external forces causing accelerations.
  ArticulatedBodyInertiaCache<T> abic;   // Articulated body inertia cache.
  std::vector<SpatialForce<T>> Zb_Bo_W;  // Articulated body biases cache.
  multibody::internal::ArticulatedBodyForceCache<T> aba_forces;  // ABA cache.
  multibody::internal::AccelerationKinematicsCache<T> ac;  // Accelerations.
};

// This class implements the interface given by DiscreteUpdateManager so that
// contact computations can be consumed by MultibodyPlant.
//
// In particular, this manager sets up a contact problem where each rigid body
// in the MultibodyPlant model is compliant without introducing state. Supported
// models include point contact with a linear model of compliance, see
// GetPointContactStiffness() and the hydroelastic contact model, see @ref
// mbp_hydroelastic_materials_properties in MultibodyPlant's Doxygen
// documentation. Dynamics of deformable bodies (if any exists) are calculated
// in DeformableDriver. Deformable body contacts are modeled as near-rigid point
// contacts where compliance is added as a means of stabilization without
// introducing additional states (i.e. the penetration distance x and its time
// derivative ẋ are not states). Dissipation is modeled using a linear model.
// For point contact, the normal contact force (in Newtons) is modeled as:
//   fₙ = k⋅(x + τ⋅ẋ)₊
// where k is the point contact stiffness, see GetPointContactStiffness(), τ is
// the dissipation timescale, and ()₊ corresponds to the "positive part"
// operator.
// Similarly, for hydroelastic contact the normal traction p (in Pascals) is:
//   p = (p₀+τ⋅dp₀/dn⋅ẋ)₊
// where p₀ is the object-centric virtual pressure field introduced by the
// hydroelastic model.
//
// TODO(amcastro-tri): Retire code from MultibodyPlant as this contact manager
// replaces all the contact related capabilities, per #16106.
//
// @warning Scalar support on T = symbolic::Expression is only limited,
// conditional to the solver in use:
//   - For TAMSI. Discrete updates are only supported when there is no contact
//     geometry. Otherwise an exception is thrown.
//   - For SAP. Discrete updates are not supported.
//
// Even when limited support for discrete updates is provided for T =
// symbolic::Expression, a MultibodyPlant can be scalar converted to symbolic in
// order to perform other supported queries, such as kinematics, or
// introspection.
//
// @tparam_default_scalar
template <typename T>
class CompliantContactManager final
    : public internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactManager)

  using internal::DiscreteUpdateManager<T>::plant;

<<<<<<< HEAD
  CompliantContactManager();
=======
  CompliantContactManager() = default;
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

  ~CompliantContactManager() final;

  // Sets the parameters to be used by the SAP solver.
  // @pre plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap.
  // @throws if called when instantiated on T = symbolic::Expression.
  void set_sap_solver_parameters(
      const contact_solvers::internal::SapSolverParameters& parameters);

  // @returns `true`.
  bool is_cloneable_to_double() const final;

  // @returns `true`.
  bool is_cloneable_to_autodiff() const final;

  // @returns `true`.
  bool is_cloneable_to_symbolic() const final;

 private:
  // TODO(amcastro-tri): Instead of friendship consider another set of class(es)
  // with tighter functionality. For instance, a class that takes care of
  // getting proximity properties and creating DiscreteContactPairs.
  friend class SapDriver<T>;
  friend class TamsiDriver<T>;

  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    systems::CacheIndex contact_kinematics;
    systems::CacheIndex discrete_contact_pairs;
    systems::CacheIndex hydroelastic_contact_info;
    systems::CacheIndex non_contact_forces_accelerations;
  };

<<<<<<< HEAD
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class CompliantContactManager;

=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  // Provide private access for unit testing only.
  friend class CompliantContactManagerTester;

  const MultibodyTreeTopology& tree_topology() const {
    return internal::GetInternalTree(this->plant()).get_topology();
  }

<<<<<<< HEAD
  std::unique_ptr<DiscreteUpdateManager<double>> CloneToDouble() const final;
  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> CloneToAutoDiffXd()
      const final;
  std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>> CloneToSymbolic()
      const final;

  // Extracts non state dependent model information from MultibodyPlant. See
  // DiscreteUpdateManager for details.
  void ExtractModelInfo() final;

  // Associates the given `DeformableModel` with `this` manager. The discrete
  // states of the deformable bodies registered in the given `model` will be
  // advanced by this manager. This manager holds onto the given pointer and
  // therefore the model must outlive the manager.
  // @throws std::exception if a deformable model has already been registered.
  // @pre model != nullptr.
  void ExtractConcreteModel(const DeformableModel<T>* model);

  // For testing purposes only, we provide a default no-op implementation on
  // arbitrary models of unknown concrete model type. Otherwise, for the closed
  // list of models forward declared in physical_model.h, we must provide a
  // function that extracts the particular variant of the physical model.
  void ExtractConcreteModel(std::monostate) {}

  void DoDeclareCacheEntries() final;

  // TODO(amcastro-tri): implement these APIs according to #16955.
  // @throws For SAP if T = symbolic::Expression.
  // @throws For TAMSI if T = symbolic::Expression only if the model contains
  // contact geometry.
=======
  // Extracts non state dependent model information from MultibodyPlant. See
  // DiscreteUpdateManager for details.
  void ExtractModelInfo() final;

  void DeclareCacheEntries() final;

  // TODO(amcastro-tri): implement these APIs according to #16955.
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final;
  void DoCalcDiscreteValues(const systems::Context<T>&,
                            systems::DiscreteValues<T>*) const final;
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final;
<<<<<<< HEAD
  void DoCalcContactResults(
      const systems::Context<T>&,
      ContactResults<T>* contact_results) const final;
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

  // This method computes sparse kinematics information for each contact pair at
  // the given configuration stored in `context`.
  std::vector<ContactPairKinematics<T>> CalcContactKinematics(
      const systems::Context<T>& context) const;

<<<<<<< HEAD
  // Eval version of CalcContactKinematics().
  const std::vector<ContactPairKinematics<T>>& EvalContactKinematics(
      const systems::Context<T>& context) const;
=======
  // Returns the dissipation time constant stored in group
  // geometry::internal::kMaterialGroup with property
  // "dissipation_time_constant". If not present, it returns
  // plant().time_step().
  T GetDissipationTimeConstant(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Helper to acquire per-geometry Coulomb friction coefficients from
  // SceneGraph. Discrete models cannot make a distinction between static and
  // dynamic coefficients of friction. Therefore this method returns the
  // coefficient of dynamic friction stored by SceneGraph while the coefficient
  // of static friction is ignored.
  // @pre id is a valid GeometryId in the inspector.
  double GetCoulombFriction(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Utility to combine stiffnesses k1 and k2 according to the rule:
  //   k  = k₁⋅k₂/(k₁+k₂)
  // In other words, the combined compliance (the inverse of stiffness) is the
  // sum of the individual compliances.
  static T CombineStiffnesses(const T& k1, const T& k2);

  // Utility to combine linear dissipation time constants. Consider two
  // spring-dampers with stiffnesses k₁ and k₂, and dissipation timescales τ₁
  // and τ₂, respectively. When these spring-dampers are connected in series,
  // they result in an equivalent spring-damper with stiffness k  =
  // k₁⋅k₂/(k₁+k₂) and dissipation τ = τ₁ + τ₂.
  // This method returns tau1 + tau2.
  static T CombineDissipationTimeConstant(const T& tau1, const T& tau2);
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

  // Given the configuration stored in `context`, this method appends discrete
  // pairs corresponding to point contact into `pairs`.
  // @pre pairs != nullptr.
  void AppendDiscreteContactPairsForPointContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method appends discrete
  // pairs corresponding to hydroelastic contact into `pairs`.
  // @pre pairs != nullptr.
  void AppendDiscreteContactPairsForHydroelasticContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method computes all
  // discrete contact pairs, including point, hydroelastic, and deformable
  // contact, into `pairs`. Contact pairs including deformable bodies are
  // guaranteed to come after point and hydroelastic contact pairs. Throws an
  // exception if `pairs` is nullptr.
  void CalcDiscreteContactPairs(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Eval version of CalcDiscreteContactPairs().
  const std::vector<internal::DiscreteContactPair<T>>& EvalDiscreteContactPairs(
      const systems::Context<T>& context) const;

  // Computes per-face contact information for the hydroelastic model (slip
  // velocity, traction, etc). On return contact_info->size() will equal the
  // number of faces discretizing the contact surface.
  void CalcHydroelasticContactInfo(
      const systems::Context<T>& context,
      std::vector<HydroelasticContactInfo<T>>* contact_info) const;

  // Eval version of CalcHydroelasticContactInfo() .
  const std::vector<HydroelasticContactInfo<T>>& EvalHydroelasticContactInfo(
      const systems::Context<T>& context) const;

  // Computes all continuous forces in the MultibodyPlant model. Joint limits
  // are not included as continuous compliant forces but rather as constraints
  // in the solver, and therefore must be excluded.
  // Values in `forces` will be overwritten.
  // @pre forces != nullptr and is consistent with plant().
  void CalcNonContactForcesExcludingJointLimits(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Computes all continuous forces in the MultibodyPlant model. Joint limits
  // are not included as continuous compliant forces but rather as constraints
  // in the solver, and therefore must be excluded.
  // Values in `forces` will be overwritten.
  // @pre forces != nullptr and is consistent with plant().
  void CalcNonContactForcesExcludingJointLimits(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Calc non-contact forces and the accelerations they induce.
  void CalcAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context,
      AccelerationsDueToExternalForcesCache<T>* no_contact_accelerations_cache)
      const;

  // Eval version of CalcAccelerationsDueToNonContactForcesCache().
  const multibody::internal::AccelerationKinematicsCache<T>&
  EvalAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context) const;

  // Helper method to fill in contact_results with point contact information
  // for the given state stored in `context`.
  // @param[in,out] contact_results is appended to
  void AppendContactResultsForPointContact(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

<<<<<<< HEAD
  // Helper method to fill in `contact_results` with hydroelastic contact
  // information for the given state stored in `context`.
  // @param[in,out] contact_results is appended to
  void AppendContactResultsForHydroelasticContact(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  CacheIndexes cache_indexes_;
  // Vector of joint damping coefficients, of size plant().num_velocities().
  // This information is extracted during the call to ExtractModelInfo().
  VectorX<T> joint_damping_;

  // deformable_driver_ computes the information on all deformable bodies needed
  // to advance the discrete states.
  std::unique_ptr<DeformableDriver<double>> deformable_driver_;

  // Specific contact solver drivers are created at ExtractModelInfo() time,
  // when the manager retrieves modeling information from MultibodyPlant.
  // Only one of these drivers will be non-nullptr.
  std::unique_ptr<SapDriver<T>> sap_driver_;
  std::unique_ptr<TamsiDriver<T>> tamsi_driver_;
=======
  // Add limit constraints for the configuration stored in `context` into
  // `problem`. Limit constraints are only added when the state q₀ for a
  // particular joint is "close" to the joint's limits (qₗ,qᵤ). To decide when
  // the state q₀ is close to the joint's limits, this method estimates a window
  // (wₗ,wᵤ) for the expected value of the configuration q at the next time
  // step. Lower constraints are considered whenever qₗ > wₗ and upper
  // constraints are considered whenever qᵤ < wᵤ. This window (wₗ,wᵤ) is
  // estimated based on the current velocity v₀ and the free motion velocities
  // v*, provided with `v_star`.
  // Since the implementation uses the current velocity v₀ to estimate whether
  // the constraint should be enabled, it is at least as good as a typical
  // continuous collision detection method. It could mispredict under conditions
  // of strong acceleration (it is assuming constant velocity across a step).
  // Still, at typical robotics step sizes and rates it would be surprising to
  // see that happen, and if it did the limit would come on in the next step.
  // TODO(amcastro-tri): Consider using the acceleration at t₀ to get a second
  // order prediction for the configuration at the next time step.
  // @pre problem must not be nullptr.
  void AddLimitConstraints(
      const systems::Context<T>& context, const VectorX<T>& v_star,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // This method takes SAP results for a given `problem` and loads forces due to
  // contact only into `contact_results`. `contact_results` is properly resized
  // on output.
  // @pre contact_results is not nullptr.
  // @pre All `num_contacts` contact constraints in `problem` were added before
  // any other SAP constraint. This requirement is imposed by this manager which
  // adds constraints (with AddContactConstraints()) to the contact problem
  // before any other constraints are added. See the implementation of
  // CalcContactProblemCache(), who is responsible for adding constraints in
  // this particular order.
  void PackContactSolverResults(
      const contact_solvers::internal::SapContactProblem<T>& problem,
      int num_contacts,
      const contact_solvers::internal::SapSolverResults<T>& sap_results,
      contact_solvers::internal::ContactSolverResults<T>* contact_results)
      const;

  CacheIndexes cache_indexes_;
  contact_solvers::internal::SapSolverParameters sap_parameters_;
  // Vector of joint damping coefficients, of size plant().num_velocities().
  // This information is extracted during the call to ExtractModelInfo().
  VectorX<T> joint_damping_;
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
};

// N.B. These geometry queries are not supported when T = symbolic::Expression
// and therefore their implementation throws.
template <>
void CompliantContactManager<symbolic::Expression>::CalcDiscreteContactPairs(
    const drake::systems::Context<symbolic::Expression>&,
    std::vector<DiscreteContactPair<symbolic::Expression>>*) const;
template <>
void CompliantContactManager<symbolic::Expression>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const drake::systems::Context<symbolic::Expression>&,
        std::vector<DiscreteContactPair<symbolic::Expression>>*) const;

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
