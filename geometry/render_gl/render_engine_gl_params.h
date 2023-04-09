#pragma once

<<<<<<< HEAD
<<<<<<<< HEAD:geometry/render_gl/render_engine_gl_params.h
#include "drake/common/name_value.h"
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace render {

/** Construction parameters for RenderEngineGl.  */
struct RenderEngineGlParams {
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
  /** Default render label to apply to a geometry when none is otherwise
   specified.  */
  RenderLabel default_label{RenderLabel::kUnspecified};

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
<<<<<<< HEAD
========
// NOLINTNEXTLINE
#warning The include path drake/geometry/render/gl_renderer/render_engine_gl_params.h is deprecated and will be removed on 2022-10-01.  Use drake/geometry/render_gl/render_engine_gl_params.h instead.

#include "drake/geometry/render_gl/render_engine_gl_params.h"
>>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c:geometry/render/gl_renderer/render_engine_gl_params.h
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
