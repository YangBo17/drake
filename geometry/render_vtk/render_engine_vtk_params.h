#pragma once

#include <optional>

<<<<<<< HEAD
#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
=======
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams  {
<<<<<<< HEAD
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_label));
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
  }

=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<render::RenderLabel> default_label{};

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  /** The rgb color to which the color buffer is cleared (each
   channel in the range [0, 1]). The default value (in byte values) would be
   [204, 229, 255].  */
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};
};

<<<<<<< HEAD
=======
namespace render {

using RenderEngineVtkParams
    DRAKE_DEPRECATED("2022-09-01", "Use the geometry namespace instead.")
    = geometry::RenderEngineVtkParams;

}  // namespace render
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
}  // namespace geometry
}  // namespace drake
