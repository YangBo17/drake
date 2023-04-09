#include "drake/multibody/parsing/parser.h"

#include <optional>
#include <set>

<<<<<<< HEAD
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
=======
#include "drake/common/filesystem.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_composite_parse.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/multibody/parsing/detail_parsing_workspace.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_select_parser.h"

namespace drake {
namespace multibody {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
<<<<<<< HEAD
=======
using internal::AddModelFromSdf;
using internal::AddModelFromUrdf;
using internal::AddModelsFromSdf;
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
using internal::CollisionFilterGroupResolver;
using internal::DataSource;
using internal::ParserInterface;
using internal::ParsingWorkspace;
using internal::SelectParser;

Parser::Parser(MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph)
    : Parser(plant, scene_graph, {}) {}

Parser::Parser(MultibodyPlant<double>* plant,
               std::string_view model_name_prefix)
    : Parser(plant, nullptr, model_name_prefix) {}

Parser::Parser(MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph,
               std::string_view model_name_prefix)
    : plant_(plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  if (!model_name_prefix.empty()) {
    model_name_prefix_ = std::string(model_name_prefix);
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  auto warnings_maybe_strict =
      [this](const DiagnosticDetail& detail) {
        if (is_strict_) {
          diagnostic_policy_.Error(detail);
        } else {
          diagnostic_policy_.WarningDefaultAction(detail);
        }
      };
  diagnostic_policy_.SetActionForWarnings(warnings_maybe_strict);
}

<<<<<<< HEAD
std::vector<ModelInstanceIndex> Parser::AddModels(
    const std::filesystem::path& file_name) {
  const std::string filename_string{file_name.string()};
  DataSource data_source(DataSource::kFilename, &filename_string);
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  return parser.AddAllModels(data_source, model_name_prefix_,
                             composite->workspace());
}

std::vector<ModelInstanceIndex> Parser::AddModelsFromUrl(
    const std::string& url) {
  const std::string file_name = internal::ResolveUri(
      diagnostic_policy_, url, package_map_, {});
  if (file_name.empty()) {
    return {};
  }
  return AddModels(file_name);
<<<<<<< HEAD
=======
namespace {
enum class FileType { kSdf, kUrdf, kMjcf };
FileType DetermineFileType(const std::string& file_name) {
  const std::string ext = filesystem::path(file_name).extension().string();
  if ((ext == ".urdf") || (ext == ".URDF")) {
    return FileType::kUrdf;
  }
  if ((ext == ".sdf") || (ext == ".SDF")) {
    return FileType::kSdf;
  }
  if ((ext == ".xml") || (ext == ".XML")) {
    return FileType::kMjcf;
  }
  throw std::runtime_error(fmt::format(
      "The file type '{}' is not supported for '{}'",
      ext, file_name));
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
=======
>>>>>>> 65b76e12737b188b94fc473aa3d3c4fb4fea5a0f
}

std::vector<ModelInstanceIndex> Parser::AddAllModelsFromFile(
    const std::string& file_name) {
<<<<<<< HEAD
  return AddModels(file_name);
}

std::vector<ModelInstanceIndex> Parser::AddModelsFromString(
    const std::string& file_contents, const std::string& file_type) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
  ParserInterface& parser = SelectParser(diagnostic_policy_, pseudo_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  return parser.AddAllModels(data_source, model_name_prefix_,
                             composite->workspace());
=======
  return CompositeAddAllModelsFromFile(file_name, {});
}

std::vector<ModelInstanceIndex> Parser::CompositeAddAllModelsFromFile(
    const std::string& file_name,
    internal::CompositeParse* composite) {
  DataSource data_source(DataSource::kFilename, &file_name);
  CollisionFilterGroupResolver resolver{plant_};
  ParsingWorkspace workspace{
    package_map_, diagnostic_policy_, plant_,
    composite ? &composite->collision_resolver() : &resolver};
  const FileType type = DetermineFileType(file_name);
  std::vector<ModelInstanceIndex> result;
  if (type == FileType::kSdf) {
    result = AddModelsFromSdf(data_source, workspace);
  } else if (type == FileType::kUrdf) {
    const std::optional<ModelInstanceIndex> maybe_model =
        AddModelFromUrdf(data_source, {}, {}, workspace);
    if (maybe_model.has_value()) {
      result = {*maybe_model};
    } else {
      throw std::runtime_error(
          fmt::format("{}: URDF model file parsing failed", file_name));
    }
  } else {  // type == FileType::kMjcf
    result = {AddModelFromMujocoXml(data_source, {}, {}, plant_)};
  }
  if (!composite) { resolver.Resolve(diagnostic_policy_); }
  return result;
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
}

ModelInstanceIndex Parser::AddModelFromFile(
    const std::string& file_name,
    const std::string& model_name) {
  return CompositeAddModelFromFile(file_name, model_name, {});
}

ModelInstanceIndex Parser::CompositeAddModelFromFile(
    const std::string& file_name,
    const std::string& model_name,
    internal::CompositeParse* composite) {
  DataSource data_source(DataSource::kFilename, &file_name);
<<<<<<< HEAD
  ParserInterface& parser = SelectParser(diagnostic_policy_, file_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  std::optional<ModelInstanceIndex> maybe_model;
  maybe_model = parser.AddModel(data_source, model_name, model_name_prefix_,
                                composite->workspace());
=======
  CollisionFilterGroupResolver resolver{plant_};
  ParsingWorkspace workspace{
    package_map_, diagnostic_policy_, plant_,
    composite ? &composite->collision_resolver() : &resolver};
  const FileType type = DetermineFileType(file_name);
  std::optional<ModelInstanceIndex> maybe_model;
  if (type == FileType::kSdf) {
    maybe_model = AddModelFromSdf(data_source, model_name, workspace);
  } else if (type == FileType::kUrdf) {
    maybe_model = AddModelFromUrdf(data_source, model_name, {}, workspace);
  } else {
    maybe_model =
        AddModelFromMujocoXml(data_source, model_name, {}, plant_);
  }
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", file_name));
  }
  if (!composite) { resolver.Resolve(diagnostic_policy_); }
  return *maybe_model;
}

ModelInstanceIndex Parser::AddModelFromString(
    const std::string& file_contents,
    const std::string& file_type,
    const std::string& model_name) {
  return CompositeAddModelFromString(file_contents, file_type, model_name, {});
}

ModelInstanceIndex Parser::CompositeAddModelFromString(
    const std::string& file_contents,
    const std::string& file_type,
    const std::string& model_name,
    internal::CompositeParse* composite) {
  DataSource data_source(DataSource::kContents, &file_contents);
  const std::string pseudo_name(data_source.GetStem() + "." + file_type);
<<<<<<< HEAD
  ParserInterface& parser = SelectParser(diagnostic_policy_, pseudo_name);
  auto composite = internal::CompositeParse::MakeCompositeParse(this);
  std::optional<ModelInstanceIndex> maybe_model;
  maybe_model = parser.AddModel(data_source, model_name, model_name_prefix_,
                                composite->workspace());
=======
  CollisionFilterGroupResolver resolver{plant_};
  ParsingWorkspace workspace{
    package_map_, diagnostic_policy_, plant_,
    composite ? &composite->collision_resolver() : &resolver};
  const FileType type = DetermineFileType(pseudo_name);
  std::optional<ModelInstanceIndex> maybe_model;
  if (type == FileType::kSdf) {
    maybe_model = AddModelFromSdf(data_source, model_name, workspace);
  } else if (type == FileType::kUrdf) {
    maybe_model = AddModelFromUrdf(data_source, model_name, {}, workspace);
  } else {  // FileType::kMjcf
    maybe_model = AddModelFromMujocoXml(data_source, model_name, {}, plant_);
  }
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  if (!maybe_model.has_value()) {
    throw std::runtime_error(
        fmt::format("{}: parsing failed", pseudo_name));
  }
  if (!composite) { resolver.Resolve(diagnostic_policy_); }
  return *maybe_model;
}

}  // namespace multibody
}  // namespace drake
