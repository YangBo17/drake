#pragma once

<<<<<<< HEAD
<<<<<<<< HEAD:geometry/render_gl/factory.h
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include <memory>

#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace drake {
namespace geometry {
namespace render {

<<<<<<< HEAD
/** Reports the availability of the RenderEngineGl implementation. */
extern const bool kHasRenderEngineGl;

/** Constructs a RenderEngine implementation which uses a purely OpenGL
 renderer. The engine only works under Ubuntu. If called on a Mac, it will
 throw.
=======
/** Constructs a RenderEngine implementation which uses a purely OpenGL
 renderer. The engine only works under Ubuntu. If called on a Mac, it will
 produce a "dummy" implementation.
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

 @note %RenderEngineGl behaves a bit differently from other RenderEngine
 implementations (e.g., RenderEngineVtk) with respect to displayed images.
 First, %RenderEngineGl can only display a *single* image type at a time. So,
 if a shown window has been requested for both label and color images, the
 images will alternate in the same window. Second, the window display draws all
 images *flipped vertically*. The image produced will be compatible with the
 Drake ecosystem, only the visualization will be upside down. This has been
 documented in https://github.com/RobotLocomotion/drake/issues/14254.

 @warning %RenderEngineGl is not threadsafe. If a SceneGraph is instantiated
 with a RenderEngineGl and there are multiple Context instances for that
<<<<<<< HEAD
 SceneGraph, rendering in multiple threads may exhibit issues.

 @throws std::exception if kHasRenderEngineGl is false. */
=======
 SceneGraph, rendering in multiple threads may exhibit issues.  */
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
std::unique_ptr<RenderEngine> MakeRenderEngineGl(
    RenderEngineGlParams params = {});

}  // namespace render
}  // namespace geometry
}  // namespace drake
<<<<<<< HEAD
========
// NOLINTNEXTLINE
#warning The include path drake/geometry/render/gl_renderer/render_engine_gl_factory.h is deprecated and will be removed on 2022-09-01.  Use drake/geometry/render_gl/factory.h instead.

#include "drake/geometry/render_gl/factory.h"
>>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c:geometry/render/gl_renderer/render_engine_gl_factory.h
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
