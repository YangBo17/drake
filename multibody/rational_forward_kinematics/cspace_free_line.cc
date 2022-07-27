#include "drake/multibody/rational_forward_kinematics/cspace_free_line.h"
#include "drake/common/symbolic_decompose.h"
namespace drake {
namespace multibody {
using solvers::MathematicalProgram;

CspaceFreeLine::CspaceFreeLine(const systems::Diagram<double>& diagram,
                               const multibody::MultibodyPlant<double>* plant,
                               const geometry::SceneGraph<double>* scene_graph,
                               SeparatingPlaneOrder plane_order)
    : CspaceFreeRegion(diagram, plant, scene_graph, plane_order,
                       CspaceRegionType::kAxisAlignedBoundingBox),
      mu_{symbolic::Variable("mu")} {
  InitializeLineVariables(plant->num_positions());
}

CspaceFreeLine::CspaceFreeLine(
    const multibody::MultibodyPlant<double>& plant,
    const std::vector<const ConvexPolytope*>& link_polytopes,
    const std::vector<const ConvexPolytope*>& obstacles,
    SeparatingPlaneOrder plane_order)
    : CspaceFreeRegion(plant, link_polytopes, obstacles, plane_order,
                       CspaceRegionType::kAxisAlignedBoundingBox),
      mu_{symbolic::Variable("mu")} {
  InitializeLineVariables(plant.num_positions());
}

std::vector<LinkVertexOnPlaneSideRational>
CspaceFreeLine::GenerateLinkOnOneSideOfPlaneRationals(
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const CspaceFreeRegion::FilteredCollisionPairs& filtered_collision_pairs)
    const {
  std::vector<LinkVertexOnPlaneSideRational> generic_rationals =
      CspaceFreeRegion::GenerateLinkOnOneSideOfPlaneRationals(
          q_star, filtered_collision_pairs);
  std::vector<LinkVertexOnPlaneSideRational> rationals;

  symbolic::Expression numerator_expr;
  symbolic::Polynomial numerator_poly;
  symbolic::Expression denominator_expr;
  symbolic::Polynomial denominator_poly;
  for (const auto& rational : generic_rationals) {
    numerator_expr = rational.rational.numerator().ToExpression();
    numerator_poly =
        symbolic::Polynomial(numerator_expr.Substitute(t_to_line_subs_map_),
                             symbolic::Variables{mu_});

    denominator_expr = rational.rational.denominator().ToExpression();
    denominator_poly =
        symbolic::Polynomial(denominator_expr.Substitute(t_to_line_subs_map_),
                             symbolic::Variables{mu_});

    rationals.emplace_back(
        symbolic::RationalFunction(numerator_poly, denominator_poly),
        rational.link_polytope, rational.expressed_body_index,
        rational.other_side_link_polytope, rational.a_A, rational.b,
        rational.plane_side, rational.plane_order);
  }
  return rationals;
}

void CspaceFreeLine::GenerateTuplesForCertification(
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const FilteredCollisionPairs& filtered_collision_pairs,
    std::vector<CspaceFreeRegion::CspacePolytopeTuple>* alternation_tuples,
    VectorX<symbolic::Variable>* lagrangian_gram_vars,
    VectorX<symbolic::Variable>* verified_gram_vars,
    VectorX<symbolic::Variable>* separating_plane_vars,
    std::vector<std::vector<int>>* separating_plane_to_tuples) const {
  ConstructTuplesInMemory(q_star, filtered_collision_pairs, 0, 1,
                          alternation_tuples, lagrangian_gram_vars,
                          verified_gram_vars, separating_plane_vars,
                          separating_plane_to_tuples);
}

AllocatedCertificationProgram CspaceFreeLine::AllocateCertificationProgram(
    const std::vector<CspaceFreeRegion::CspacePolytopeTuple>&
        alternation_tuples,
    const VectorX<symbolic::Variable>& separating_plane_vars,
    const VerificationOption& option) const {
  auto prog = std::make_unique<solvers::MathematicalProgram>();

  // Adds decision variables.
  prog->AddDecisionVariables(separating_plane_vars);
  symbolic::Variables mu_variables{mu_};
  prog->AddIndeterminates(solvers::VectorIndeterminate<1>(mu_));

  // a map to store polynomials which will have s0 and s1 in its coefficients
  // for rapidly updating the coefficients for every new certification program
  std::unordered_map<
      symbolic::Polynomial,
      std::unordered_map<symbolic::Monomial,
                         solvers::Binding<solvers::LinearEqualityConstraint>>>
      polynomial_to_monomial_to_binding_map;

  symbolic::Expression coefficient_expression;
  // For each rational numerator, add the constraint that the Lagrangian
  // polynomials >= 0, and the verified polynomial >= 0.
  //
  // Recall that a univariate polynomial p(μ) is nonnegative on [0, 1] if and
  // only if p(μ) = λ(μ) + ν(μ)*μ*(1-μ) if deg(p) = 2d with deg(λ) ≤ 2d and
  // deg(ν) ≤ 2d - 2 p(μ) = λ(μ)*μ + ν(μ)*(1-μ) if deg(p) = 2d + 1 with deg(λ) ≤
  // 2d and deg(ν) ≤ 2d and λ, ν are SOS
  for (const auto& tuple : alternation_tuples) {
    symbolic::Polynomial verified_polynomial = tuple.rational_numerator;
    verified_polynomial.SetIndeterminates({mu_});
    int d = verified_polynomial.TotalDegree() / 2;
    auto [l, Ql] =
        prog->NewSosPolynomial(mu_variables, 2 * d, option.lagrangian_type);
    if (verified_polynomial.TotalDegree() % 2 == 0) {
      auto [v, Qv] = prog->NewSosPolynomial(mu_variables, 2 * d - 2,
                                            option.lagrangian_type);
      verified_polynomial -= l + v * mu_ * symbolic::Polynomial(1) - mu_;
    } else {
      auto [v, Qv] =
          prog->NewSosPolynomial(mu_variables, 2 * d, option.lagrangian_type);
      verified_polynomial -= l * mu_ + v * symbolic::Polynomial(1) - mu_;
    }
    // now add s0 and s1 to the indeterminates to enable us to add them to the
    // program
    VectorX<symbolic::Variable> mu_s0_s1_variables{1 + s0_.size() + s1_.size()};
    mu_s0_s1_variables[0] = mu_;
    int ctr = 1;
    for (int i = 0; i < s0_.size(); i++) {
      mu_s0_s1_variables[ctr] = s0_[i];
      ++ctr;
    }
    for (int i = 0; i < s1_.size(); i++) {
      mu_s0_s1_variables[ctr] = s1_[i];
      ++ctr;
    }

    // preallocate linear equality constraints for the zero equality awaiting
    // substitution of s0 and s1 also create a map from the monomial to the
    // appropriate linear equality constraint
    std::unordered_map<symbolic::Monomial,
                       solvers::Binding<solvers::LinearEqualityConstraint>>
        monomial_to_equality_constraint;
    for (const auto& [monomial, coefficient] :
         verified_polynomial.monomial_to_coefficient_map()) {
      // construct dummy expression to ensure that the binding has the appropriate variables
      coefficient_expression = 0;
      for(const auto& v: coefficient.GetVariables())
      {
        coefficient_expression += v;
      }
      monomial_to_equality_constraint.insert(
          {monomial, (prog->AddLinearEqualityConstraint(coefficient_expression, 0))});
    }

    polynomial_to_monomial_to_binding_map.insert(
        {verified_polynomial, monomial_to_equality_constraint});
  }
  return {std::move(prog), polynomial_to_monomial_to_binding_map};
}

void CspaceFreeLine::InitializeLineVariables(int num_positions) {
  s0_.resize(num_positions);
  s1_.resize(num_positions);
  for (int i = 0; i < num_positions; ++i) {
    s0_[i] = drake::symbolic::Variable("s0[" + std::to_string(i) + "]");
    s1_[i] = drake::symbolic::Variable("s1[" + std::to_string(i) + "]");
  }

  const drake::VectorX<drake::symbolic::Variable> t =
      (this->rational_forward_kinematics()).t();
  for (int i = 0;
       i < ((this->rational_forward_kinematics()).plant().num_positions());
       ++i) {
    t_to_line_subs_map_[t[i]] = mu_ * s0_[i] + (1 - mu_) * s1_[i];
  }
}

AllocatedCertificationProgram::AllocatedCertificationProgram(
    std::unique_ptr<solvers::MathematicalProgram> prog,
    std::unordered_map<
        symbolic::Polynomial,
        std::unordered_map<symbolic::Monomial,
                           solvers::Binding<solvers::LinearEqualityConstraint>>>
        polynomial_to_monomial_to_bindings_map)
    : prog_{std::move(prog)},
      polynomial_to_monomial_to_bindings_map_{
          polynomial_to_monomial_to_bindings_map} {};

void AllocatedCertificationProgram::
    EvaluatePolynomialsAndUpdateProgCoefficients(symbolic::Environment env) {
  solvers::LinearEqualityConstraint* constraint;
  symbolic::Expression coefficient;
  for (auto& [polynomial, monomial_to_binding] :
       polynomial_to_monomial_to_bindings_map_) {
    symbolic::Polynomial evaluated_polynomial = polynomial.EvaluatePartial(env);
    for (auto& [monomial, binding] : monomial_to_binding) {
      constraint = binding.evaluator().get();
      coefficient =
          evaluated_polynomial.monomial_to_coefficient_map().at(monomial);
      std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
      for (int i = 0; i < binding.variables().size(); ++i) {
        // I believe that the variables in binding are sorted in the order of
        // the linear constraint. TODO(hongkai.dai) check me on this
        map_var_to_index[binding.variables()[i].get_id()] = i;
      }
      Eigen::RowVectorXd A{binding.variables().size()};
      double b;
      symbolic::DecomposeAffineExpression(coefficient, map_var_to_index, A, b);
    }
  }
}

}  // namespace multibody
}  // namespace drake