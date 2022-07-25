#include "drake/multibody/rational_forward_kinematics/cspace_free_line.h"

namespace drake {
namespace multibody {

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
  std::unordered_map<symbolic::Variable, symbolic::Expression> subs;
  const drake::VectorX<drake::symbolic::Variable> t =
      (this->rational_forward_kinematics()).t();
  for (int i = 0;
       i < ((this->rational_forward_kinematics()).plant().num_positions());
       ++i) {
    subs[t[i]] = mu_ * s0_[i] + (1 - mu_) * s1_[i];
  }

  symbolic::Expression numerator_expr;
  symbolic::Polynomial numerator_poly;
  symbolic::Expression denominator_expr;
  symbolic::Polynomial denominator_poly;
  for (const auto& rational : generic_rationals) {
    numerator_expr = rational.rational.numerator().ToExpression();
    numerator_poly = symbolic::Polynomial(numerator_expr.Substitute(subs));

    denominator_expr = rational.rational.denominator().ToExpression();
    denominator_poly = symbolic::Polynomial(denominator_expr.Substitute(subs));

    rationals.emplace_back(
        symbolic::RationalFunction(numerator_poly, denominator_poly),
        rational.link_polytope, rational.expressed_body_index,
        rational.other_side_link_polytope, rational.a_A, rational.b,
        rational.plane_side, rational.plane_order);
  }
  return rationals;
}

void CspaceFreeLine::InitializeLineVariables(int num_positions) {
  s0_.resize(num_positions);
  s1_.resize(num_positions);
  for (int i = 0; i < num_positions; ++i) {
    s0_[i] = drake::symbolic::Variable("s0[" + std::to_string(i) + "]");
    s1_[i] = drake::symbolic::Variable("s1[" + std::to_string(i) + "]");
  }
}

}  // namespace multibody
}  // namespace drake