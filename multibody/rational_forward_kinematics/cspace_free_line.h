#pragma once
#include "drake/multibody/rational_forward_kinematics/cspace_free_region.h"

namespace drake {
namespace multibody {
// a class for storing the allocated certification program and constructing the
// final program
class AllocatedCertificationProgram {
 public:
  AllocatedCertificationProgram(
      std::unique_ptr<solvers::MathematicalProgram> prog,
      std::unordered_map<
          symbolic::Polynomial,
          std::unordered_map<
              symbolic::Monomial,
              solvers::Binding<solvers::LinearEqualityConstraint>>> polynomial_to_monomial_to_bindings_map);

  void EvaluatePolynomialsAndUpdateProgCoefficients(symbolic::Environment env);

 private:
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  std::unordered_map<
      symbolic::Polynomial,
      std::unordered_map<symbolic::Monomial,
                         solvers::Binding<solvers::LinearEqualityConstraint>>>
      polynomial_to_monomial_to_bindings_map_;
};

/**
 * This class is designed to certify lines in the configuration space
 * parametrized as μ*s₀ + (1−μ)*s₁. This is a special case of certifying a
 * 1-dimensional polytope but we provide this implementation to enable certain
 * optimizations.
 */
class CspaceFreeLine : public CspaceFreeRegion {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreeLine)

  CspaceFreeLine(const systems::Diagram<double>& diagram,
                 const multibody::MultibodyPlant<double>* plant,
                 const geometry::SceneGraph<double>* scene_graph,
                 SeparatingPlaneOrder plane_order);

  CspaceFreeLine(const multibody::MultibodyPlant<double>& plant,
                 const std::vector<const ConvexPolytope*>& link_polytopes,
                 const std::vector<const ConvexPolytope*>& obstacles,
                 SeparatingPlaneOrder plane_order);

  void GenerateTuplesForCertification(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const FilteredCollisionPairs& filtered_collision_pairs,
      std::vector<CspaceFreeRegion::CspacePolytopeTuple>* alternation_tuples,
      VectorX<symbolic::Variable>* lagrangian_gram_vars,
      VectorX<symbolic::Variable>* verified_gram_vars,
      VectorX<symbolic::Variable>* separating_plane_vars,
      std::vector<std::vector<int>>* separating_plane_to_tuples) const;

  AllocatedCertificationProgram AllocateCertificationProgram(
      const std::vector<CspaceFreeRegion::CspacePolytopeTuple>&
          alternation_tuples,
      const VectorX<symbolic::Variable>& separating_plane_vars,
      const VerificationOption& option) const;

  std::unique_ptr<solvers::MathematicalProgram> AllocateCertificationProgram(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const CspaceFreeRegion::FilteredCollisionPairs& filtered_collision_pairs)
      const;

  const symbolic::Variable get_mu() const { return mu_; }
  const drake::VectorX<drake::symbolic::Variable> get_s0() const { return s0_; }
  const drake::VectorX<drake::symbolic::Variable> get_s1() const { return s1_; }

  std::vector<LinkVertexOnPlaneSideRational>
  GenerateLinkOnOneSideOfPlaneRationals(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const FilteredCollisionPairs& filtered_collision_pairs) const override;

 private:
  // the variable of the line going from 0 to 1
  const symbolic::Variable mu_;
  // the symbolic start of the free line
  drake::VectorX<drake::symbolic::Variable> s0_;
  // the symbolic end of the free line
  drake::VectorX<drake::symbolic::Variable> s1_;
  // map for performing substitution from t to μ*s₀ + (1−μ)*s₁
  std::unordered_map<symbolic::Variable, symbolic::Expression>
      t_to_line_subs_map_;

  void InitializeLineVariables(int num_positions);
};

}  // namespace multibody
}  // namespace drake
