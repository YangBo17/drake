#include "drake/geometry/render_gl/factory.h"

#include <utility>

#include "drake/geometry/render_gl/internal_render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

<<<<<<< HEAD
// Definition of extern bool in factory.h. When we build against *this* .cc file
// RenderEngineGl is always available.
const bool kHasRenderEngineGl = true;

=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
std::unique_ptr<RenderEngine> MakeRenderEngineGl(RenderEngineGlParams params) {
  return std::make_unique<internal::RenderEngineGl>(std::move(params));
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
