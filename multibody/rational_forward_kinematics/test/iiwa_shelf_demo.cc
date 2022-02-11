#include <iostream>
#include <limits>

#include "drake/common/find_resource.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/polytope_cover.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rational_forward_kinematics/cspace_free_region.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
const double kInf = std::numeric_limits<double>::infinity();

class IiwaDiagram {
 public:
  // @param num_iiwas We use num_iiwas != 1 purely for visualizaing multiple
  // postures of the same IIWA simultaneously. Don't use num_iiwas!=1 for
  // computing cspace region
  IiwaDiagram(int num_iiwas = 1)
      : meshcat_{std::make_shared<geometry::Meshcat>()} {
    systems::DiagramBuilder<double> builder;
    auto [plant, sg] = AddMultibodyPlantSceneGraph(&builder, 0.);
    plant_ = &plant;
    scene_graph_ = &sg;

    multibody::Parser parser(plant_);
    const std::string iiwa_file_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_coarse_collision.sdf");
    for (int iiwa_count = 0; iiwa_count < num_iiwas; ++iiwa_count) {
      const auto iiwa_instance = parser.AddModelFromFile(
          iiwa_file_path, fmt::format("iiwa{}", iiwa_count));
      plant_->WeldFrames(plant_->world_frame(),
                         plant_->GetFrameByName("iiwa_link_0", iiwa_instance));
      iiwa_instances_.push_back(iiwa_instance);

      const std::string schunk_file_path = FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf/"
          "schunk_wsg_50_welded_fingers.sdf");

      const Frame<double>& link7 =
          plant_->GetFrameByName("iiwa_link_7", iiwa_instance);
      const math::RigidTransformd X_7G(
          math::RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
          Eigen::Vector3d(0, 0, 0.114));
      const auto wsg_instance = parser.AddModelFromFile(
          schunk_file_path, fmt::format("gripper{}", iiwa_count));
      const auto& schunk_frame = plant_->GetFrameByName("body", wsg_instance);
      plant_->WeldFrames(link7, schunk_frame, X_7G);
      // SceneGraph should ignore the collision between any geometries on the
      // gripper, and between the gripper and link 6
      geometry::GeometrySet gripper_link6_geometries;
      auto add_gripper_geometries =
          [this, wsg_instance,
           &gripper_link6_geometries](const std::string& body_name) {
            const geometry::FrameId frame_id = plant_->GetBodyFrameIdOrThrow(
                plant_->GetBodyByName(body_name, wsg_instance).index());
            gripper_link6_geometries.Add(frame_id);
          };
      add_gripper_geometries("body");
      add_gripper_geometries("left_finger");
      add_gripper_geometries("right_finger");

      const geometry::FrameId link_6_frame_id = plant_->GetBodyFrameIdOrThrow(
          plant_->GetBodyByName("iiwa_link_6", iiwa_instance).index());
      const auto& inspector = scene_graph_->model_inspector();
      const std::vector<geometry::GeometryId> link_6_geometries =
          inspector.GetGeometries(link_6_frame_id, geometry::Role::kProximity);
      for (const auto geometry : link_6_geometries) {
        gripper_link6_geometries.Add(geometry);
      }

      scene_graph_->collision_filter_manager().Apply(
          geometry::CollisionFilterDeclaration().ExcludeWithin(
              gripper_link6_geometries));

      // Ignore collision between IIWA links.
      std::vector<geometry::GeometryId> iiwa_geometry_ids;
      for (const auto& body_index : plant_->GetBodyIndices(iiwa_instance)) {
        const std::vector<geometry::GeometryId> body_geometry_ids =
            plant_->GetCollisionGeometriesForBody(plant_->get_body(body_index));
        iiwa_geometry_ids.insert(iiwa_geometry_ids.end(),
                                 body_geometry_ids.begin(),
                                 body_geometry_ids.end());
      }
      scene_graph_->collision_filter_manager().Apply(
          geometry::CollisionFilterDeclaration().ExcludeWithin(
              geometry::GeometrySet(iiwa_geometry_ids)));
    }

    const std::string shelf_file_path =
        FindResourceOrThrow("drake/sos_iris_certifier/shelves.sdf");
    const auto shelf_instance =
        parser.AddModelFromFile(shelf_file_path, "shelves");
    const auto& shelf_frame =
        plant_->GetFrameByName("shelves_body", shelf_instance);
    const math::RigidTransformd X_WShelf(Eigen::Vector3d(0.8, 0, 0.4));
    plant_->WeldFrames(plant_->world_frame(), shelf_frame, X_WShelf);

    plant_->Finalize();

    geometry::MeshcatVisualizerParams meshcat_params{};
    meshcat_params.role = geometry::Role::kIllustration;
    visualizer_ = &geometry::MeshcatVisualizer<double>::AddToBuilder(
        &builder, *scene_graph_, meshcat_, meshcat_params);
    diagram_ = builder.Build();
  }

  const systems::Diagram<double>& diagram() const { return *diagram_; }

  const multibody::MultibodyPlant<double>& plant() const { return *plant_; }

  const geometry::SceneGraph<double>& scene_graph() const {
    return *scene_graph_;
  }

  const std::vector<ModelInstanceIndex>& iiwa_instances() const {
    return iiwa_instances_;
  }

 private:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::shared_ptr<geometry::Meshcat> meshcat_;
  geometry::MeshcatVisualizer<double>* visualizer_;
  std::vector<ModelInstanceIndex> iiwa_instances_;
};

Eigen::VectorXd FindInitialPosture(const MultibodyPlant<double>& plant,
                                   systems::Context<double>* plant_context) {
  InverseKinematics ik(plant, plant_context);
  const auto& link7 = plant.GetFrameByName("iiwa_link_7");
  const auto& shelf = plant.GetFrameByName("shelves_body");
  ik.AddPositionConstraint(link7, Eigen::Vector3d::Zero(), shelf,
                           Eigen::Vector3d(-0.4, -0.2, -0.2),
                           Eigen::Vector3d(-.1, 0.2, 0.2));
  ik.AddMinimumDistanceConstraint(0.02);

  Eigen::Matrix<double, 7, 1> q_init;
  q_init << 0.1, 0.3, 0.2, 0.5, 0.4, 0.3, 0.2;
  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_init);
  const auto result = solvers::Solve(ik.prog());
  if (!result.is_success()) {
    drake::log()->warn("Cannot find the posture\n");
  }
  return result.GetSolution(ik.q());
}

void BuildCandidateCspacePolytope(const Eigen::VectorXd q_free,
                                  Eigen::MatrixXd* C, Eigen::VectorXd* d) {
  const int C_rows = 26;
  C->resize(C_rows, 7);
  // Create arbitrary polytope normals.
  // clang-format off
  (*C) << 0.5, 0.3, 0.2, -0.1, -1, 0, 0.5,
          -0.1, 0.4, 0.2, 0.1, 0.5, -0.2, 0.3,
          0.4, 1.2, -0.3, 0.2, 0.1, 0.4, 0.5,
          -0.5, -2, -1.5, 0.3, 0.6, 0.1, -0.2,
          0.2, 0.1, -0.5, 0.3, 0.4, 1.4, 0.5,
          0.1, -0.5, 0.4, 1.5, -0.3, 0.2, 0.1,
          0.2, 0.3, 1.3, 0.2, -0.3, -0.5, -0.2,
          1.4, 0.1, -0.1, 0.2, -0.3, 0.1, 0.5,
          -1.1, 0.2, 0.3, -0.1, 0.5, 0.2, -0.1,
          0.2, -0.3, -1.2, 0.5, -0.3, 0.1, 0.3,
          0.2, -1.5, 0.1, 0.4, -0.3, -0.2, 0.6,
          0.1, 0.4, -0.2, 0.3, 0.9, -0.5, 0.8,
          -0.2, 0.3, -0.1, 0.8, -0.4, 0.2, 1.4,
          0.1, -0.2, 0.2, -0.3, 1.2, -0.3, 0.1,
          0.3, -0.1, 0.2, 0.5, -0.3, -2.1, 1.2,
          0.4, -0.3, 1.5, -0.3, 1.8, -0.1, 0.4,
          1.2, -0.3, 0.4, 0.8, 1.2, -0.4, -0.8,
          0.4, -0.2, 0.5, 1.4, 0.7, -0.2, -0.9,
          -0.1, 0.4, -0.2, 0.3, 1.5, 0.1, -0.6,
          -0.1, -0.3, 0.2, 1.1, -1.2, 1.3, 2.1,
          0.1, -0.4, 0.2, 1.3, 1.2, 0.3, -1.1,
          0.1, -1.4, 0.2, 0.3, 0.2, 0.3, -0.7,
          -0.3, -0.5, 0.4, -1.5, -0.2, 1.3, -2.1,
          0.5, 1.2, -0.3, 1.2, 0.5, -2.1, 0.4,
          -0.2, 2.1, 3.2, 1.5, 0.3, -1.8, 0.3,
          -2.1, 0.3, 2.0, 0.2, 1.3, -0.5, -0.6;
  // clang-format on
  for (int i = 0; i < C_rows; ++i) {
    C->row(i).normalize();
  }
  *d = (*C) * (q_free / 2).array().tan().matrix() +
       0.0001 * Eigen::VectorXd::Ones(C_rows);
  if (!geometry::optimization::HPolyhedron(*C, *d).IsBounded()) {
    throw std::runtime_error("C*t <= d is not bounded");
  }
}

void SearchCspacePolytope(
    const std::string& write_file,
    const std::optional<std::string>& read_file = std::nullopt) {
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  const IiwaDiagram iiwa_diagram{};
  auto diagram_context = iiwa_diagram.diagram().CreateDefaultContext();
  auto& plant_context =
      iiwa_diagram.plant().GetMyMutableContextFromRoot(diagram_context.get());
  const auto q0 = FindInitialPosture(iiwa_diagram.plant(), &plant_context);
  iiwa_diagram.plant().SetPositions(&plant_context, q0);
  iiwa_diagram.diagram().Publish(*diagram_context);
  std::cout << "Type to continue";
  std::cin.get();

  Eigen::MatrixXd C_init;
  Eigen::VectorXd d_init;
  if (read_file.has_value()) {
    Eigen::VectorXd t_lower_dummy, t_upper_dummy;
    ReadCspacePolytopeFromFile(read_file.value(), &C_init, &d_init,
                               &t_lower_dummy, &t_upper_dummy);
  } else {
    BuildCandidateCspacePolytope(q0, &C_init, &d_init);
  }

  const CspaceFreeRegion dut(iiwa_diagram.diagram(), &(iiwa_diagram.plant()),
                             &(iiwa_diagram.scene_graph()),
                             SeparatingPlaneOrder::kAffine,
                             CspaceRegionType::kGenericPolytope);

  CspaceFreeRegion::FilteredCollisionPairs filtered_collision_pairs{};

  CspaceFreeRegion::BinarySearchOption binary_search_option{
      .epsilon_max = 0.05,
      .epsilon_min = 0.,
      .max_iters = 4,
      .compute_polytope_volume = true,
      .num_threads = -1};
  solvers::SolverOptions solver_options;
  solver_options.SetOption(solvers::CommonSolverOption::kPrintToConsole, false);
  CspaceFreeRegionSolution cspace_free_region_solution;
  Eigen::VectorXd q_star = Eigen::Matrix<double, 7, 1>::Zero();
  dut.CspacePolytopeBinarySearch(
      q_star, filtered_collision_pairs, C_init, d_init, binary_search_option,
      solver_options, q0, std::nullopt, &cspace_free_region_solution);
  CspaceFreeRegion::BilinearAlternationOption bilinear_alternation_option{
      .max_iters = 50,
      .convergence_tol = 0.001,
      .lagrangian_backoff_scale = 0.002,
      .redundant_tighten = 0.5,
      .compute_polytope_volume = true,
      .num_threads = -1};

  std::vector<double> polytope_volumes, ellipsoid_determinants;
  dut.CspacePolytopeBilinearAlternation(
      q_star, filtered_collision_pairs, cspace_free_region_solution.C,
      cspace_free_region_solution.d, bilinear_alternation_option,
      solver_options, q0, std::nullopt, &cspace_free_region_solution,
      &polytope_volumes, &ellipsoid_determinants);
  Eigen::MatrixXd C_final(cspace_free_region_solution.C);
  Eigen::VectorXd d_final(cspace_free_region_solution.d);
  Eigen::MatrixXd P_final(cspace_free_region_solution.P);
  Eigen::VectorXd q_final(cspace_free_region_solution.q);

  // Compute the determinant of the final polytope.
  drake::log()->info("det(P) {}", P_final.determinant());

  const Eigen::VectorXd t_lower =
      (iiwa_diagram.plant().GetPositionLowerLimits().array() / 2)
          .tan()
          .matrix();
  const Eigen::VectorXd t_upper =
      (iiwa_diagram.plant().GetPositionUpperLimits().array() / 2)
          .tan()
          .matrix();

  WriteCspacePolytopeToFile(C_final, d_final, t_lower, t_upper, write_file, 10);
  drake::log()->info("polytope volumes {}",
                     Eigen::Map<Eigen::RowVectorXd>(polytope_volumes.data(),
                                                    polytope_volumes.size()));
  drake::log()->info(
      "ellipsoid determinants {}",
      Eigen::Map<Eigen::RowVectorXd>(ellipsoid_determinants.data(),
                                     ellipsoid_determinants.size()));
}

void VisualizePostures(const std::string& read_file) {
  Eigen::MatrixXd C;
  Eigen::VectorXd d, t_lower, t_upper;
  ReadCspacePolytopeFromFile(read_file, &C, &d, &t_lower, &t_upper);
  const int num_postures = 2;
  // Solve a program such that the sum of inter-posture distance is maximized.
  solvers::MathematicalProgram prog;
  auto t = prog.NewContinuousVariables(7, num_postures);
  for (int i = 0; i < num_postures; ++i) {
    prog.AddLinearConstraint(C, Eigen::VectorXd::Constant(d.rows(), -kInf), d,
                             t.col(i));
    prog.AddBoundingBoxConstraint(t_lower, t_upper, t.col(i));
  }
  // Now add the cost max (t.col(i) - t.col(j))^2
  for (int i = 0; i < num_postures; ++i) {
    for (int j = i + 1; j < num_postures; ++j) {
      // A * [t.col(i); t.col(j)] = t.col(i) - t.col(j)
      Eigen::MatrixXd A(7, 14);
      A << Eigen::MatrixXd::Identity(7, 7), -Eigen::MatrixXd::Identity(7, 7);
      prog.AddQuadraticCost(-A.transpose() * A, Eigen::VectorXd::Zero(14),
                            {t.col(i), t.col(j)}, false /* is_convex=false */);
    }
    prog.SetInitialGuess(t.col(i), Eigen::VectorXd::Random(7));
  }
  const auto result = solvers::Solve(prog);
  std::cout << result.get_solution_result() << "\n";
  DRAKE_DEMAND(result.is_success());

  Eigen::MatrixXd t_sol(7, num_postures);
  for (int i = 0; i < num_postures; ++i) {
    t_sol.col(i) = result.GetSolution(t.col(i));
  }
  Eigen::MatrixXd q_sol = 2 * (t_sol.array().atan().matrix());
  std::cout << "postures\n" << q_sol.transpose() << "\n";

  IiwaDiagram iiwa_diagram(num_postures);
  auto diagram_context = iiwa_diagram.diagram().CreateDefaultContext();
  auto& plant_context =
      iiwa_diagram.plant().GetMyMutableContextFromRoot(diagram_context.get());
  for (int i = 0; i < num_postures; ++i) {
    iiwa_diagram.plant().SetPositions(
        &plant_context, iiwa_diagram.iiwa_instances()[i], q_sol.col(i));
  }
  iiwa_diagram.diagram().Publish(*diagram_context);
  std::cout << "Type to continue\n";
  std::cin.get();
}

int DoMain() {
  const std::string write_file = "iiwa_shelf_cspace_polytope4.txt";
  // const std::string read_file = "iiwa_shelf_cspace_polytope2.txt";
  SearchCspacePolytope(write_file);
  // VisualizePostures(write_file);
  return 0;
}
}  // namespace multibody
}  // namespace drake

int main() { return drake::multibody::DoMain(); }
