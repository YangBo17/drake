#pragma once
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "drake/multibody/rational_forward_kinematics/cspace_free_region.h"

namespace drake {
namespace multibody {

namespace internal {
// Checks whether two polynomials are equal. Needed to build maps with
// Polynomials as keys in the AllocateProgram method
struct ComparePolynomials {
  bool operator()(const symbolic::Polynomial& p1,
                  const symbolic::Polynomial& p2) const {
    return p1.CoefficientsAlmostEqual(p2, 1E-12);
  }
};

// a class for storing the allocated certification program and constructing the
// final program
class AllocatedCertificationProgram {
 public:
  AllocatedCertificationProgram() = default;

  AllocatedCertificationProgram(
      std::unique_ptr<solvers::MathematicalProgram> prog,
      std::unordered_map<
          symbolic::Polynomial,
          std::unordered_map<
              symbolic::Monomial,
              solvers::Binding<solvers::LinearEqualityConstraint>>,
          std::hash<symbolic::Polynomial>, internal::ComparePolynomials>
          polynomial_to_monomial_to_bindings_map);

  void EvaluatePolynomialsAndUpdateProgram(symbolic::Environment env);

  solvers::MathematicalProgramResult solve(
      const solvers::SolverOptions& solver_options);

  const solvers::MathematicalProgram* get_prog() const { return prog_.get(); }
  const std::unordered_map<
      symbolic::Polynomial,
      std::unordered_map<symbolic::Monomial,
                         solvers::Binding<solvers::LinearEqualityConstraint>>,
      std::hash<symbolic::Polynomial>, internal::ComparePolynomials>
  get_polynomial_to_monomial_to_bindings_map() const {
    return polynomial_to_monomial_to_bindings_map_;
  }

 private:
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  std::unordered_map<
      symbolic::Polynomial,
      std::unordered_map<symbolic::Monomial,
                         solvers::Binding<solvers::LinearEqualityConstraint>>,
      std::hash<symbolic::Polynomial>, internal::ComparePolynomials>
      polynomial_to_monomial_to_bindings_map_;
};
}  // namespace internal

class CspaceLineTuple {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceLineTuple)

  CspaceLineTuple(const symbolic::Variable& mu,
                  const drake::VectorX<symbolic::Variable>& s0,
                  const drake::VectorX<symbolic::Variable>& s1,
                  const symbolic::Polynomial& m_rational_numerator,
                  const VerificationOption& option);

  void Evaluate_s0_s1_map(const symbolic::Environment& env);

  void Evaluate_s0_s1(const Eigen::Ref<const Eigen::VectorXd>& s0,
                      const Eigen::Ref<const Eigen::VectorXd>& s1);

  const symbolic::Polynomial get_rational_numerator() const {
    return rational_numerator_;
  }
  const solvers::MathematicalProgram* get_prog() const { return &prog_; }
  const symbolic::Polynomial get_lambda() const { return lambda_; }
  const solvers::MatrixXDecisionVariable get_Q_lambda() const {
    return Q_lambda_;
  }
  const solvers::MatrixXDecisionVariable get_Q_lambda_lower_part() const {
    return Q_lambda_lower_part_;
  }
  const symbolic::Polynomial get_nu() const { return nu_; }
  const solvers::MatrixXDecisionVariable get_Q_nu() const { return Q_nu_; }
  const solvers::MatrixXDecisionVariable get_Q_nu_lower_part() const {
    return Q_nu_lower_part_;
  }
  const symbolic::Polynomial get_p() const { return p_; }
  const solvers::VectorXDecisionVariable get_separating_planes_variables()
      const {
    return separating_plane_variables_;
  }

  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>
  get_psatz_bindings() const {
    return psatz_bindings_;
  }

 private:
  // This is the numerator of the rational aᵀx+b-δ or -δ-aᵀx-b which we will
  // verify is positive
  symbolic::Polynomial rational_numerator_;

  // program that stores the list of constraints required to certify this tuple
  solvers::MathematicalProgram prog_;

  //  a univariate polynomial p(μ) is nonnegative on [0, 1] if and
  // only if p(μ) = λ(μ) + ν(μ)*μ*(1-μ) if deg(p) = 2d with deg(λ) ≤ 2d and
  // deg(ν) ≤ 2d - 2 p(μ) = λ(μ)*μ + ν(μ)*(1-μ) if deg(p) = 2d + 1 with
  // deg(λ) ≤ 2d and deg(ν) ≤ 2d and λ, ν are SOS. These are those polynomials
  // and their associated Gram matrices.
  symbolic::Polynomial lambda_;
  solvers::MatrixXDecisionVariable Q_lambda_;
  // the lower triangular part of Q_lambda_ for ease of adding to programs
  solvers::VectorXDecisionVariable Q_lambda_lower_part_;
  symbolic::Polynomial nu_;
  solvers::MatrixXDecisionVariable Q_nu_;
  // the lower triangular part of Q_nu_ for ease of adding to programs
  solvers::VectorXDecisionVariable Q_nu_lower_part_;
  symbolic::Polynomial p_;

  symbolic::Variables s_vars_;
  drake::VectorX<drake::symbolic::Variable> s0_;
  // the symbolic end of the free line
  drake::VectorX<drake::symbolic::Variable> s1_;

  solvers::VectorXDecisionVariable separating_plane_variables_;

  // bindings for p(μ) - λ(μ) + ν(μ)*μ*(1-μ) = 0
  std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>
      psatz_bindings_;
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
                 SeparatingPlaneOrder plane_order,
                 std::optional<Eigen::VectorXd> q_star,
                 const FilteredCollisionPairs& filtered_collision_pairs = {},
                 const VerificationOption& option = {});

  void GenerateTuplesForCertification(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const FilteredCollisionPairs& filtered_collision_pairs,
      std::list<CspaceLineTuple>* tuples,
      VectorX<symbolic::Variable>* separating_plane_vars,
      std::vector<std::vector<int>>* separating_plane_to_tuples,
      std::vector<
          std::vector<solvers::Binding<solvers::LorentzConeConstraint>>>*
          separating_plane_to_lorentz_cone_constraints) const;

  void GenerateTuplesForCertification(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const FilteredCollisionPairs& filtered_collision_pairs,
      std::vector<CspaceFreeLine::CspacePolytopeTuple>* alternation_tuples,
      VectorX<symbolic::Variable>* lagrangian_gram_vars,
      VectorX<symbolic::Variable>* verified_gram_vars,
      VectorX<symbolic::Variable>* separating_plane_vars,
      std::vector<std::vector<int>>* separating_plane_to_tuples,
      std::vector<
          std::vector<solvers::Binding<solvers::LorentzConeConstraint>>>*
          separating_plane_to_lorentz_cone_constraints) const;

  const symbolic::Variable get_mu() const { return mu_; }
  const drake::VectorX<drake::symbolic::Variable> get_s0() const { return s0_; }
  const drake::VectorX<drake::symbolic::Variable> get_s1() const { return s1_; }
  const std::list<CspaceLineTuple>* get_tuples() const { return &tuples_; }
  const VectorX<symbolic::Variable> get_separating_plane_vars() const {
    return separating_plane_vars_;
  }

  std::vector<LinkOnPlaneSideRational> GenerateRationalsForLinkOnOneSideOfPlane(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const FilteredCollisionPairs& filtered_collision_pairs) const override;

  /**
   * Updates the constraints in tuples_ for a new set of endpoints.
   */
  void EvaluateTuplesForEndpoints(const Eigen::Ref<const Eigen::VectorXd>& s0,
                                  const Eigen::Ref<const Eigen::VectorXd>& s1);

  /**
   * Certifies whether the line μ*s₀ + (1−μ)*s₁. If the return is true, a formal
   * proof of non-collision is generated. If the result is false there may be no
   * collisions but this fact cannot be certified with the constructed SOS
   * program.
   * @param s0
   * @param s1
   * @param solver_options
   * @return
   */
  bool CertifyTangentConfigurationSpaceLine(
      const Eigen::Ref<const Eigen::VectorXd>& s0,
      const Eigen::Ref<const Eigen::VectorXd>& s1,
      std::vector<SeparatingPlane<double>>* separating_planes_sol,
      const solvers::SolverOptions& solver_options = solvers::SolverOptions());
  /**
   * Adds the constraint that all of the tuples in the @param i separating plane
   * are on the appropriate side of the plane to @param prog.
   * Assumes that @param prog contains the separating plane variables already
   * and has μ set as the indeterminate already.
   */
  void AddCertifySeparatingPlaneConstraintToProg(
      solvers::MathematicalProgram* prog, int i) const;

 protected:
  /**
   * runs the certification program for a separating plane. Returns true if the
   * program succeeds in certification. If it succeeds then separating_plane_sol
   * is filled with the solution.
   */
  bool CertifyPlane(int i, SeparatingPlane<double>* separating_plane_sol,
                    const solvers::SolverOptions& solver_options =
                        solvers::SolverOptions()) const;

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
  // q_star_ for stereographic projection
  Eigen::VectorXd q_star_;

  FilteredCollisionPairs filtered_collision_pairs_;

  VerificationOption option_;

  // preallocated certification program
  internal::AllocatedCertificationProgram allocated_certification_program_;

  std::list<CspaceLineTuple> tuples_;
  VectorX<symbolic::Variable> separating_plane_vars_;
  /*
   * tuples can be grouped
   * based on the separating planes. separating_plane_to_tuples[i] are the
   * indices in alternation_tuples such that these tuples are all for
   * this->separating_planes()[i].
   */
  std::vector<std::vector<int>> separating_plane_to_tuples_;
  std::vector<std::vector<solvers::Binding<solvers::LorentzConeConstraint>>>
      separating_plane_to_lorentz_cone_constraints_;

  void InitializeLineVariables(int num_positions);

  internal::AllocatedCertificationProgram AllocateCertificationProgram() const;
};

}  // namespace multibody
}  // namespace drake
