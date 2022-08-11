#include "drake/multibody/rational_forward_kinematics/cspace_free_line.h"

#include <execution>
#include <iostream>
#include <utility>

#include "drake/common/symbolic/decompose.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"
#include <mutex>
namespace drake {
namespace multibody {
using solvers::MathematicalProgram;

namespace {
// TODO(Alex.Amice) THIS is COPIED FROM cspace_free_region.cc. Resolve this with
// Hongkai
SeparatingPlane<double> GetSeparatingPlaneSolution(
    const SeparatingPlane<symbolic::Variable>& plane,
    const solvers::MathematicalProgramResult& result) {
  symbolic::Environment env;
  const Eigen::VectorXd decision_variables_val =
      result.GetSolution(plane.decision_variables);
  env.insert(plane.decision_variables, decision_variables_val);
  Vector3<symbolic::Expression> a_sol;
  for (int i = 0; i < 3; ++i) {
    a_sol(i) = plane.a(i).EvaluatePartial(env);
  }
  const symbolic::Expression b_sol = plane.b.EvaluatePartial(env);
  return SeparatingPlane<double>(
      a_sol, b_sol, plane.positive_side_geometry, plane.negative_side_geometry,
      plane.expressed_link, plane.order, decision_variables_val);
}
}  // namespace

CspaceLineTuple::CspaceLineTuple(
    const symbolic::Variable& mu, const drake::VectorX<symbolic::Variable>& s0,
    const drake::VectorX<symbolic::Variable>& s1,
    const symbolic::Polynomial& m_rational_numerator,
    const VerificationOption& option)
    : rational_numerator_{m_rational_numerator},
      prog_{solvers::MathematicalProgram()},
      p_{m_rational_numerator},
      s_vars_{},
      s0_{s0},
      s1_{s1} {
  // check that m_rational_numerator was passed with the expected indeterminates
  symbolic::Variables rational_numerator_indeterminates_expected{mu};

  // this map will be used to construct the default evaluation environment for
  // s0 and s1 later. This is put here to avoid looping over the set of
  // variables twice.
  std::unordered_map<symbolic::Variable, double> map;
  for (int i = 0; i < s0.size(); ++i) {
    map.insert({s0[i], 0});
    rational_numerator_indeterminates_expected.insert(s0[i]);
  }
  for (int i = 0; i < s1.size(); ++i) {
    map.insert({s1[i], 0});
    rational_numerator_indeterminates_expected.insert(s1[i]);
  }

  DRAKE_ASSERT(rational_numerator_indeterminates_expected.IsSupersetOf(
      rational_numerator_.indeterminates()));

  s_vars_ = rational_numerator_indeterminates_expected;
  s_vars_.erase(mu);
  prog_.AddIndeterminates(solvers::VectorIndeterminate<1>(mu));

  // get the separating planes variables from the rational_numerator_
  separating_plane_variables_.resize(
      rational_numerator_.decision_variables().size());
  int i = 0;
  for (const auto& var : rational_numerator_.decision_variables()) {
    separating_plane_variables_[i] = var;
    ++i;
  }
  prog_.AddDecisionVariables(separating_plane_variables_);
  p_.SetIndeterminates({mu});

  auto extract_lower_part =
      [](const solvers::MatrixXDecisionVariable& sym_mat) {
        const int size = sym_mat.rows() * (sym_mat.rows() + 1) / 2;
        solvers::VectorXDecisionVariable vect;
        vect.resize(size);
        int count = 0;
        for (int c = 0; c < static_cast<int>(sym_mat.rows()); ++c) {
          for (int r = c; r < static_cast<int>(sym_mat.rows()); ++r) {
            vect[count] = sym_mat(r, c);
            ++count;
          }
        }
        return vect;
      };
  // construct the Lagrangian variables and the Psatz equality constraints
  if (p_.TotalDegree() == 0) {
    for (const auto& [monomial, coeff] : p_.monomial_to_coefficient_map()) {
      prog_.AddLinearConstraint(coeff >= 0);
    }
  } else {
    int d = p_.TotalDegree() / 2;
    auto [l, Ql] =
        prog_.NewSosPolynomial({mu}, 2 * d, option.lagrangian_type, "Sl");
    lambda_ = l;
    Q_lambda_ = Ql;
    Q_lambda_lower_part_ = extract_lower_part(Q_lambda_);

    if (p_.TotalDegree() % 2 == 0) {
      auto [v, Qv] =
          prog_.NewSosPolynomial({mu}, 2 * d - 2, option.lagrangian_type, "Sv");
      nu_ = v;
      Q_nu_ = Qv;
      Q_nu_lower_part_ = extract_lower_part(Q_nu_);
      p_ -= lambda_ + nu_ * mu * (symbolic::Polynomial(1, {mu}) - mu);
    } else {
      auto [v, Qv] =
          prog_.NewSosPolynomial({mu}, 2 * d, option.lagrangian_type, "Sv");
      nu_ = v;
      Q_nu_ = Qv;
      Q_nu_lower_part_ = extract_lower_part(Q_nu_);
      p_ -= lambda_ * mu + nu_ * (symbolic::Polynomial(1, {mu}) - mu);
    }
  }
  psatz_bindings_ = prog_.AddEqualityConstraintBetweenPolynomials(
      p_.EvaluatePartial(symbolic::Environment{map}), symbolic::Polynomial());
}

void CspaceLineTuple::Evaluate_s0_s1_map(const symbolic::Environment& env) {
  // TODO(Alex.Amice) is this check worth doing?
  DRAKE_DEMAND(env.domain() == s_vars_);

  for (const auto& binding : psatz_bindings_) {
    prog_.RemoveConstraint(binding);
  }
  psatz_bindings_ = prog_.AddEqualityConstraintBetweenPolynomials(
      p_.EvaluatePartial(env), symbolic::Polynomial());
}

void CspaceLineTuple::Evaluate_s0_s1(
    const Eigen::Ref<const Eigen::VectorXd>& s0,
    const Eigen::Ref<const Eigen::VectorXd>& s1) {
  DRAKE_DEMAND(s0.size() == s0_.size());
  DRAKE_DEMAND(s1.size() == s1_.size());
  symbolic::Environment env;
  for (int i = 0; i < s0.size(); ++i) {
    env.insert(s0_[i], s0[i]);
    env.insert(s1_[i], s1[i]);
  }
  Evaluate_s0_s1_map(env);
}

CspaceFreeLine::CspaceFreeLine(
    const systems::Diagram<double>& diagram,
    const multibody::MultibodyPlant<double>* plant,
    const geometry::SceneGraph<double>* scene_graph,
    SeparatingPlaneOrder plane_order, std::optional<Eigen::VectorXd> q_star,
    const FilteredCollisionPairs& filtered_collision_pairs,
    const VerificationOption& option)
    : CspaceFreeRegion(diagram, plant, scene_graph, plane_order,
                       CspaceRegionType::kAxisAlignedBoundingBox),
      mu_{symbolic::Variable("mu")},
      filtered_collision_pairs_{filtered_collision_pairs},
      option_{option} {
  if (q_star.has_value()) {
    DRAKE_DEMAND(q_star.value().size() == plant->num_positions());
    q_star_ = q_star.value();
  } else {
    q_star_ = Eigen::VectorXd::Zero(plant->num_positions());
  }

  InitializeLineVariables(plant->num_positions());

  // allocate all the tuples
  GenerateTuplesForCertification(
      q_star_, filtered_collision_pairs_, &tuples_, &separating_plane_vars_,
      &separating_plane_to_tuples_,
      &separating_plane_to_lorentz_cone_constraints_);
}

void CspaceFreeLine::EvaluateTuplesForEndpoints(
    const Eigen::Ref<const Eigen::VectorXd>& s0,
    const Eigen::Ref<const Eigen::VectorXd>& s1) {
  std::for_each(std::execution::par, tuples_.begin(), tuples_.end(),
                [s0, s1](auto&& tuple) { tuple.Evaluate_s0_s1(s0, s1); });
}

bool CspaceFreeLine::CertifyTangentConfigurationSpaceLine(
    const Eigen::Ref<const Eigen::VectorXd>& s0,
    const Eigen::Ref<const Eigen::VectorXd>& s1,
    std::vector<SeparatingPlane<double>>* separating_planes_sol,
    const solvers::SolverOptions& solver_options) {
  auto clock_start = std::chrono::system_clock::now();
  EvaluateTuplesForEndpoints(s0, s1);
  auto clock_now = std::chrono::system_clock::now();
  drake::log()->info(fmt::format(
      "Bindings updated in {} s",
      static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                             clock_now - clock_start)
                             .count()) /
          1000));
  std::vector<bool> is_success(separating_planes().size());
  separating_planes_sol->resize(separating_planes().size());

  // TODO(Alex.Amice) parallelize this
  clock_start = std::chrono::system_clock::now();
//  std::for_each(std::execution::par_unseq, separating_planes().begin(),
//                separating_planes().end(),
//                [&is_success, &separating_planes_sol, &solver_options,
//                 this](SeparatingPlane<symbolic::Variable>& plane) {
//                  int plane_index =
//                      static_cast<int>(&plane - &(separating_planes()[0]));
//                  is_success[plane_index] = this->CertifyPlane(
//                      plane_index, &(separating_planes_sol->at(plane_index)),
//                      solver_options);
//                });

//    for (const auto& plane : separating_planes()) {
//      int plane_index = static_cast<int>(&plane - &(separating_planes()[0]));
//      is_success[plane_index] = this->CertifyPlane(
//          plane_index, &(separating_planes_sol->at(plane_index)),
//          solver_options);
//    }
//  bool ret = std::all_of(is_success.begin(), is_success.end(),
//                         [](bool val) { return val; });

  // make one big program
    solvers::MathematicalProgram prog = solvers::MathematicalProgram();
    prog.AddDecisionVariables(separating_plane_vars_);
    for(int i = 0; i < static_cast<int>(separating_planes().size()); ++i){
      AddCertifySeparatingPlaneConstraintToProg(&prog, i);
    }
    auto result = solvers::Solve(prog, std::nullopt, solver_options);
    if (result.is_success()) {
      for(int i = 0; i < static_cast<int>(separating_planes().size()); ++i) {
        separating_planes_sol->at(i) = GetSeparatingPlaneSolution(
            this->separating_planes()[i], result);
      }
    }
   bool ret = result.is_success();

  clock_now = std::chrono::system_clock::now();
  drake::log()->info(fmt::format(
      "certified in {} s",
      static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                             clock_now - clock_start)
                             .count()) /
          1000));
  return ret;
}

bool CspaceFreeLine::CertifyTangentConfigurationSpaceLine(
      const Eigen::Ref<const Eigen::MatrixXd>& s0,
      const Eigen::Ref<const Eigen::MatrixXd>& s1,
      std::vector<std::vector<SeparatingPlane<double>>>* separating_planes_sol_per_row,
      const solvers::SolverOptions& solver_options) {
  DRAKE_DEMAND(s0.rows() == s1.rows());
  DRAKE_DEMAND(s0.cols() == s1.cols());
  auto certify_line = [&s0, &s1, &separating_planes_sol_per_row, &solver_options, this](int row_idx) {
    solvers::MathematicalProgram prog = solvers::MathematicalProgram();
    prog.AddDecisionVariables(separating_plane_vars_);
    // prevents data race on the tuple information
    update_bindings_mutex_.lock();
    EvaluateTuplesForEndpoints(s0.row(row_idx), s1.row(row_idx));
    for(int i = 0; i < static_cast<int>(this->separating_planes().size()); ++i){
      AddCertifySeparatingPlaneConstraintToProg(&prog, i);
    }
    update_bindings_mutex_.unlock();
    auto result = solvers::Solve(prog, std::nullopt, solver_options);
    if (result.is_success()) {
      for(int i = 0; i < static_cast<int>(separating_planes().size()); ++i) {
        separating_planes_sol_per_row->at(row_idx).at(i) = GetSeparatingPlaneSolution(
            this->separating_planes()[i], result);
      }
    }
    return result.is_success()
  };
  // TODO(Alex.Amice) COMPLETE THIS FUNCTION.
  return true
}

std::vector<LinkOnPlaneSideRational>
CspaceFreeLine::GenerateRationalsForLinkOnOneSideOfPlane(
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const CspaceFreeRegion::FilteredCollisionPairs& filtered_collision_pairs)
    const {
  std::vector<LinkOnPlaneSideRational> generic_rationals =
      CspaceFreeRegion::GenerateRationalsForLinkOnOneSideOfPlane(
          q_star, filtered_collision_pairs);
  std::vector<LinkOnPlaneSideRational> rationals;

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
        rational.link_geometry, rational.expressed_body_index,
        rational.other_side_link_geometry, rational.a_A, rational.b,
        rational.plane_side, rational.plane_order,
        rational.lorentz_cone_constraints);
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
    std::vector<std::vector<int>>* separating_plane_to_tuples,
    std::vector<std::vector<solvers::Binding<solvers::LorentzConeConstraint>>>*
        separating_plane_to_lorentz_cone_constraints) const {
  ConstructTuples(q_star, filtered_collision_pairs, 0, 1, alternation_tuples,
                  lagrangian_gram_vars, verified_gram_vars,
                  separating_plane_vars, separating_plane_to_tuples,
                  separating_plane_to_lorentz_cone_constraints);
}

void CspaceFreeLine::GenerateTuplesForCertification(
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const FilteredCollisionPairs& filtered_collision_pairs,
    std::list<CspaceLineTuple>* tuples,
    VectorX<symbolic::Variable>* separating_plane_vars,
    std::vector<std::vector<int>>* separating_plane_to_tuples,
    std::vector<std::vector<solvers::Binding<solvers::LorentzConeConstraint>>>*
        separating_plane_to_lorentz_cone_constraints) const {
  // Build tuples.
  const auto rationals = GenerateRationalsForLinkOnOneSideOfPlane(
      q_star, filtered_collision_pairs);

  separating_plane_to_tuples->resize(
      static_cast<int>(this->separating_planes().size()));
  separating_plane_to_tuples->resize(3);

  for (const auto& rational : rationals) {
    tuples->emplace_back(mu_, s0_, s1_, rational.rational.numerator(), option_);
    (*separating_plane_to_tuples)
        [this->map_geometries_to_separating_planes().at(
             SortedPair<geometry::GeometryId>(
                 rational.link_geometry->id(),
                 rational.other_side_link_geometry->id()))]
            .push_back(tuples->size() - 1);
  }

  // Set separating_plane_vars.
  int separating_plane_vars_count = 0;
  for (const auto& separating_plane : this->separating_planes()) {
    separating_plane_vars_count += separating_plane.decision_variables.rows();
  }
  separating_plane_vars->resize(separating_plane_vars_count);
  separating_plane_vars_count = 0;
  for (const auto& separating_plane : this->separating_planes()) {
    separating_plane_vars->segment(separating_plane_vars_count,
                                   separating_plane.decision_variables.rows()) =
        separating_plane.decision_variables;
    separating_plane_vars_count += separating_plane.decision_variables.rows();
  }
  // Set the separating plane lorentz cone constraints.
  separating_plane_to_lorentz_cone_constraints->clear();
  separating_plane_to_lorentz_cone_constraints->resize(
      this->separating_planes().size());
  for (const auto& rational : rationals) {
    if (!rational.lorentz_cone_constraints.empty()) {
      const int plane_index = this->map_geometries_to_separating_planes().at(
          SortedPair<geometry::GeometryId>(
              rational.link_geometry->id(),
              rational.other_side_link_geometry->id()));
      (*separating_plane_to_lorentz_cone_constraints)[plane_index].insert(
          (*separating_plane_to_lorentz_cone_constraints)[plane_index].end(),
          rational.lorentz_cone_constraints.begin(),
          rational.lorentz_cone_constraints.end());
    }
  }
}

void CspaceFreeLine::AddCertifySeparatingPlaneConstraintToProg(
    solvers::MathematicalProgram* prog, int i) const {
  // tuples_ is a list and there are typically more of these than indices in
  // separating_plane_to_tuples_[i] so it is faster to step through the list in
  // this outer loop.
  int ctr = 0;
  const std::vector<int> tuple_indices = separating_plane_to_tuples_[i];
  //  symbolic::Variables prog_variables{prog->decision_variables()};
  for (const auto& tuple : tuples_) {
    // if this index is one we need
    if (std::find(tuple_indices.begin(), tuple_indices.end(), ctr) !=
        tuple_indices.end()) {
      prog->AddDecisionVariables(tuple.get_Q_lambda_lower_part());
      prog->AddDecisionVariables(tuple.get_Q_nu_lower_part());
      for (const auto& binding : tuple.get_prog()->GetAllConstraints()) {
        prog->AddConstraint(binding);
      }
    }
    ++ctr;
  }
}

bool CspaceFreeLine::CertifyPlane(
    int plane_index, SeparatingPlane<double>* separating_plane_sol,
    const solvers::SolverOptions& solver_options) const {
  solvers::MathematicalProgram prog = solvers::MathematicalProgram();
  prog.AddDecisionVariables(separating_plane_vars_);
  AddCertifySeparatingPlaneConstraintToProg(&prog, plane_index);
  auto result = solvers::Solve(prog, std::nullopt, solver_options);
  if (result.is_success()) {
    *separating_plane_sol = GetSeparatingPlaneSolution(
        this->separating_planes()[plane_index], result);
  }
  return result.is_success();
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


}  // namespace multibody
}  // namespace drake
