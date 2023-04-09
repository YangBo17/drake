#include "drake/multibody/parsing/detail_sdf_diagnostic.h"

<<<<<<< HEAD
#include <utility>

=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

<<<<<<< HEAD
SDFormatDiagnostic::SDFormatDiagnostic(
    const drake::internal::DiagnosticPolicy* diagnostic,
    const drake::multibody::internal::DataSource* data_source,
    const std::string& file_extension)
    : diagnostic_(diagnostic), data_source_(data_source),
      file_extension_(file_extension) {
  DRAKE_DEMAND(diagnostic != nullptr);
  DRAKE_DEMAND(data_source != nullptr);
}

DiagnosticDetail SDFormatDiagnostic::MakeDetail(
    const sdf::Element& element, const std::string& message) const {
  DiagnosticDetail detail;
  if (!element.FilePath().empty()) {
        detail.filename = element.FilePath();
  } else {
    if (data_source_->IsFilename()) {
      detail.filename = data_source_->GetAbsolutePath();
    } else {
      detail.filename = data_source_->GetStem() + "." + file_extension_;
    }
  }
  detail.line = element.LineNumber();
  detail.message = message;
  return detail;
}

void SDFormatDiagnostic::Warning(
    sdf::ElementConstPtr element, const std::string& message) const {
  diagnostic_->Warning(MakeDetail(*element, message));
}

void SDFormatDiagnostic::Error(
    sdf::ElementConstPtr element, const std::string& message) const {
  diagnostic_->Error(MakeDetail(*element, message));
}

DiagnosticPolicy SDFormatDiagnostic::MakePolicyForNode(
    const sdf::Element& element) const {
  DiagnosticPolicy result;
  result.SetActionForWarnings(
      [this, &element](const DiagnosticDetail& detail) {
        diagnostic_->Warning(MakeDetail(element, detail.message));
      });
  result.SetActionForErrors(
      [this, &element](const DiagnosticDetail& detail) {
        diagnostic_->Error(MakeDetail(element, detail.message));
      });
  return result;
}

bool SDFormatDiagnostic::PropagateErrors(
    const sdf::Errors& errors) const {
  bool result = false;
  for (const auto& e : errors) {
    DiagnosticDetail detail;
    detail.filename = e.FilePath();
    detail.line = e.LineNumber();
    if (e.XmlPath().has_value()) {
      detail.message = fmt::format(
          "At XML path {}: {}", e.XmlPath().value(), e.Message());
    } else {
      detail.message = e.Message();
    }
    if (IsError(e)) {
      diagnostic_->Error(detail);
      result = true;
    } else {
      diagnostic_->Warning(detail);
    }
  }
  return result;
}

bool IsError(const sdf::Error& report) {
  switch (report.Code()) {
    case sdf::ErrorCode::ELEMENT_DEPRECATED:
    case sdf::ErrorCode::VERSION_DEPRECATED:
    case sdf::ErrorCode::NONE: {
      return false;
    }
    default: {
      return true;
    }
  }
}

bool PropagateErrors(
    sdf::Errors&& input_errors,
    sdf::Errors* output_errors) {
  bool result = false;
  for (auto& e : input_errors) {
    if (IsError(e)) {
      result = true;
    }
    output_errors->push_back(std::move(e));
  }
  return result;
}

void CheckSupportedElements(
    const SDFormatDiagnostic& diagnostic,
    sdf::ElementConstPtr root_element,
=======
void CheckSupportedElements(
    const DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    const std::set<std::string>& supported_elements) {
  CheckSupportedElements(diagnostic, root_element.get(), supported_elements);
}

void CheckSupportedElements(
<<<<<<< HEAD
    const SDFormatDiagnostic& diagnostic,
=======
    const DiagnosticPolicy& diagnostic,
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    const sdf::Element* root_element,
    const std::set<std::string>& supported_elements) {
  DRAKE_DEMAND(root_element != nullptr);

<<<<<<< HEAD
  sdf::ElementConstPtr element = root_element->GetFirstElement();
=======
  sdf::ElementPtr element = root_element->GetFirstElement();
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  while (element) {
    const std::string& element_name = element->GetName();
    if ((supported_elements.find(element_name) == supported_elements.end()) &&
        element->GetExplicitlySetInFile()) {
<<<<<<< HEAD
      // Unsupported elements in the drake namespace are errors.
      if (element_name.find("drake:") == 0) {
        std::string message =
            std::string("Unsupported SDFormat element in ") +
            root_element->GetName() + std::string(": ") + element_name;
        diagnostic.Error(element, std::move(message));
      } else {
        std::string message =
            std::string("Ignoring unsupported SDFormat element in ") +
            root_element->GetName() + std::string(": ") + element_name;
        diagnostic.Warning(element, std::move(message));
=======
      internal::DiagnosticDetail detail;
      if (!element->FilePath().empty()) {
        detail.filename = element->FilePath();
      }
      detail.line = element->LineNumber();
      detail.message =
          std::string("Unsupported SDFormat element in ") +
          root_element->GetName() + std::string(": ") + element_name;
      // Unsupported elements in the drake namespace are errors.
      if (element_name.find("drake:") == 0) {
        diagnostic.Error(detail);
      } else {
        diagnostic.Warning(detail);
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
      }
    }
    element = element->GetNextElement();
  }
}

void CheckSupportedElementValue(
<<<<<<< HEAD
    const SDFormatDiagnostic& diagnostic,
    sdf::ElementConstPtr root_element,
=======
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
    const std::string& element_name,
    const std::string& expected) {
  DRAKE_DEMAND(root_element != nullptr);

  if (!root_element->HasElement(element_name)) {
    return;
  }

<<<<<<< HEAD
  sdf::ElementConstPtr element = root_element->FindElement(element_name);
=======
  sdf::ElementPtr element = root_element->GetElement(element_name);
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  if (!element->GetExplicitlySetInFile()) {
    return;
  }

  sdf::ParamPtr value = element->GetValue();
  if (value->GetAsString() != expected) {
<<<<<<< HEAD
    std::string message =
      std::string("Unsupported value for SDFormat element ") +
        element->GetName() + std::string(": ") + value->GetAsString();
    diagnostic.Warning(element, message);
=======
    internal::DiagnosticDetail detail;
    if (!element->FilePath().empty()) {
      detail.filename = element->FilePath();
    }
    detail.line = element->LineNumber();
    detail.message =
        std::string("Unsupported value for SDFormat element ") +
        element->GetName() + std::string(": ") + value->GetAsString();
    diagnostic.Warning(detail);
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
