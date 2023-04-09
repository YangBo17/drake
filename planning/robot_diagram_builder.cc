#include "drake/planning/robot_diagram_builder.h"

#include <stdexcept>

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {

using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using systems::DiagramBuilder;
using systems::System;

template <typename T>
RobotDiagramBuilder<T>::RobotDiagramBuilder(double time_step)
    : builder_(std::make_unique<DiagramBuilder<T>>()),
      pair_(AddMultibodyPlantSceneGraph<T>(builder_.get(), time_step)),
      plant_(pair_.plant),
      scene_graph_(pair_.scene_graph),
      parser_(&plant_) {}

template <typename T>
RobotDiagramBuilder<T>::~RobotDiagramBuilder() = default;

template <typename T>
<<<<<<< HEAD
<<<<<<< HEAD
std::unique_ptr<RobotDiagram<T>> RobotDiagramBuilder<T>::Build() {
  ThrowIfAlreadyBuilt();
  if (!plant().is_finalized()) {
    plant().Finalize();
=======
std::unique_ptr<RobotDiagram<T>> RobotDiagramBuilder<T>::BuildDiagram() {
  ThrowIfAlreadyBuilt();
  if (!IsPlantFinalized()) {
    FinalizePlant();
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
std::unique_ptr<RobotDiagram<T>> RobotDiagramBuilder<T>::Build() {
  ThrowIfAlreadyBuilt();
  if (!plant().is_finalized()) {
    plant().Finalize();
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
  }
  return std::unique_ptr<RobotDiagram<T>>(
      new RobotDiagram<T>(std::move(builder_)));
}

template <typename T>
bool RobotDiagramBuilder<T>::IsDiagramBuilt() const {
  if (builder_ == nullptr) {
    return true;
  }
  if (builder_->already_built()) {
    throw std::logic_error(
        "RobotDiagramBuilder: Do not call mutable_builder().Build() to create"
<<<<<<< HEAD
<<<<<<< HEAD
        " a Diagram; instead, call Build() to create a RobotDiagram.");
=======
        " a Diagram; instead, call BuildDiagram() to create a RobotDiagram.");
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
        " a Diagram; instead, call Build() to create a RobotDiagram.");
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
  }
  return false;
}

template <typename T>
void RobotDiagramBuilder<T>::ThrowIfAlreadyBuilt() const {
  if (IsDiagramBuilt()) {
    throw std::logic_error(
        "RobotDiagramBuilder: Build() has already been called to create a"
        " RobotDiagram; this RobotDiagramBuilder may no longer be used.");
  }
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::planning::RobotDiagramBuilder)
