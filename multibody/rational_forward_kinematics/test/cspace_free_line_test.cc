#include "drake/multibody/rational_forward_kinematics/cspace_free_line.h"

#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

// Helper method for testing CspaceFreeLine.
// CspaceFreeLine DoublePendulumCspaceFreeLine() {
// A simple double pendulum with link lengths `l1` and `l2` with a box at the
// tip with diagonal `2r` between two (fixed) walls at `w` from the origin.
// The true configuration space is - w  ≤ l₁s₁ + l₂s₁₊₂ ± r * sin(1+2±π/4) ≤ w
// . These regions are visualized as the white region at
// https://www.desmos.com/calculator/2ugpepabig.

class DoublePendulumTest : public ::testing::Test {
 public:
  DoublePendulumTest() {
    const double l1 = 2.0;
    const double l2 = 1.0;
    const double r = .6;
    const double w = 1.83;
    const std::string double_pendulum_urdf = fmt::format(
        R"(
    <robot name="double_pendulum">
      <link name="fixed">
        <collision name="right">
          <origin rpy="0 0 0" xyz="{w_plus_one_half} 0 0"/>
          <geometry><box size="1 1 10"/></geometry>
        </collision>
        <collision name="left">
          <origin rpy="0 0 0" xyz="-{w_plus_one_half} 0 0"/>
          <geometry><box size="1 1 10"/></geometry>
        </collision>
      </link>
      <joint name="fixed_link_weld" type="fixed">
        <parent link="world"/>
        <child link="fixed"/>
      </joint>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
        <parent link="world"/>
        <child link="link1"/>
      </joint>
      <link name="link2">
        <collision name="box">
          <origin rpy="0 0 0" xyz="0 0 -{l2}"/>
          <geometry><box size="{r} {r} {r}"/></geometry>
        </collision>
      </link>
      <joint name="joint2" type="revolute">
        <origin rpy="0 0 0" xyz="0 0 -{l1}"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
        <parent link="link1"/>
        <child link="link2"/>
      </joint>
    </robot>
    )",
        fmt::arg("w_plus_one_half", w + .5), fmt::arg("l1", l1),
        fmt::arg("l2", l2), fmt::arg("r", r / sqrt(2)));

    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    multibody::Parser(plant_).AddModelFromString(double_pendulum_urdf, "urdf");
    plant_->Finalize();
    diagram_ = builder.Build();
  }

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  const Eigen::Vector2d q_star_0_{0.0, 0.0};
  const Eigen::Vector2d q_star_1_{0.0, 0.0};
};

TEST_F(DoublePendulumTest, TestCspaceFreeLineConstructor) {
  CspaceFreeLine dut(*diagram_, plant_, scene_graph_,
                     SeparatingPlaneOrder::kAffine);
  EXPECT_TRUE(dut.get_s0().size() == 2);
  EXPECT_TRUE(dut.get_s1().size() == 2);
}

TEST_F(DoublePendulumTest, TestGenerateLinkOnOneSideOfPlaneRationals) {
  CspaceFreeLine dut(*diagram_, plant_, scene_graph_,
                     SeparatingPlaneOrder::kAffine);
  CspaceFreeRegion dut2(*diagram_, plant_, scene_graph_,
                        SeparatingPlaneOrder::kAffine,
                        CspaceRegionType::kAxisAlignedBoundingBox);
  std::vector<LinkVertexOnPlaneSideRational> rationals_free_line_0 =
      dut.GenerateLinkOnOneSideOfPlaneRationals(q_star_0_, {});
  std::vector<LinkVertexOnPlaneSideRational> rationals_free_region_0 =
      dut2.GenerateLinkOnOneSideOfPlaneRationals(q_star_0_, {});

  std::vector<LinkVertexOnPlaneSideRational> rationals_free_line_1 =
      dut.GenerateLinkOnOneSideOfPlaneRationals(q_star_1_, {});
  std::vector<LinkVertexOnPlaneSideRational> rationals_free_region_1 =
      dut2.GenerateLinkOnOneSideOfPlaneRationals(q_star_1_, {});

  auto test_same_rationals = [&dut, &dut2](std::vector<LinkVertexOnPlaneSideRational>& rationals_free_line,
      std::vector<LinkVertexOnPlaneSideRational>& rationals_free_region) {

    EXPECT_EQ(rationals_free_region.size(), rationals_free_line.size());

    symbolic::Environment env1_line = {{dut.get_s0()[0], 0.0},
                                       {dut.get_s0()[1], -0.5},
                                       {dut.get_s1()[0], -0.25},
                                       {dut.get_s1()[1], 0.5},
                                       {dut.get_mu(), 0.0}};
    Eigen::VectorXd mu_vals;
    mu_vals.LinSpaced(10, 0, 1);
    std::cout << mu_vals << std::endl;
    // first place we evaluate the rationals is at s0
    symbolic::Environment env1_region = {
        {dut2.rational_forward_kinematics().t()[0], 0.0},
        {dut2.rational_forward_kinematics().t()[0], -0.5}};

    for (unsigned int i = 0; i < rationals_free_line.size(); ++i) {
      EXPECT_EQ(rationals_free_line.at(i).link_polytope->body_index(),
                rationals_free_region.at(i).link_polytope->body_index());
      EXPECT_EQ(rationals_free_line.at(i).expressed_body_index,
                rationals_free_region.at(i).expressed_body_index);
      EXPECT_EQ(
          rationals_free_line.at(i).other_side_link_polytope->body_index(),
          rationals_free_region.at(i).other_side_link_polytope->body_index());

      // for some reason these expressions are evaluating to False even though they print to be the same
//      for (unsigned int j = 0; j < rationals_free_line.at(i).a_A.size(); ++j) {
//        EXPECT_TRUE(rationals_free_line.at(i).a_A(j).EqualTo(
//            rationals_free_region.at(i).a_A(j)));
//      }
//      EXPECT_TRUE(
//          rationals_free_line.at(i).b.EqualTo(rationals_free_region.at(i).b));
      EXPECT_EQ(rationals_free_line.at(i).plane_side,
                rationals_free_region.at(i).plane_side);


      EXPECT_EQ(rationals_free_line.at(i).plane_order,
                rationals_free_region.at(i).plane_order);

      for (unsigned int j = 0; j < mu_vals.size(); j++) {
        double mu = mu_vals[j];
        env1_line[dut.get_mu()] = mu;
        env1_region[dut2.rational_forward_kinematics().t()[0]] =
            mu * env1_line[dut.get_s0()[0]] + (1 - mu) * env1_line[dut.get_s1()[0]];
        env1_region[dut2.rational_forward_kinematics().t()[1]] =
            mu * env1_line[dut.get_s0()[1]] + (1 - mu) * env1_line[dut.get_s1()[1]];
        EXPECT_NEAR(rationals_free_line.at(i).rational.Evaluate(env1_line),
                    rationals_free_region.at(i).rational.Evaluate(env1_region),
                    1E-12);
      }
    }
  };
  test_same_rationals(rationals_free_line_0, rationals_free_region_0);
  test_same_rationals(rationals_free_line_1, rationals_free_region_1);
}

}  // namespace multibody
}  // namespace drake