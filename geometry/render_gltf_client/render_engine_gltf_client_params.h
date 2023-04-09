#pragma once

#include <optional>
#include <string>

<<<<<<< HEAD
#include "drake/common/name_value.h"
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Construction parameters for the MakeRenderEngineGltfClient() to create a
 client as part of the @ref render_engine_gltf_client_server_api. */
struct RenderEngineGltfClientParams {
<<<<<<< HEAD
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(base_url));
    a->Visit(DRAKE_NVP(render_endpoint));
    a->Visit(DRAKE_NVP(default_label));
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(cleanup));
  }

=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  /** The base url of the server communicate with.
   See GetUrl() for details. */
  std::string base_url{"http://127.0.0.1:8000"};

  /** (Advanced) The server endpoint to retrieve renderings from.
   See GetUrl() for details. */
  std::string render_endpoint{"render"};

  /** The (optional) label to apply when none is otherwise specified. */
  std::optional<render::RenderLabel> default_label{};

  /** Whether or not the client should log information about which files are
   being generated, as well as any information about HTTP communications between
   the client and server such as HTTP header information, url and port, etc.
   Information is logged at the debug level, so your application will need to
<<<<<<< HEAD
   logging::set_log_level() to `"debug"`.  @sa drake/common/text_logging.h */
  bool verbose = false;

  /** Whether or not the client should cleanup files generated / retrieved from
   the server.  By default (`cleanup=true`), after a server image response has
   been loaded into the client's memory, the glTF scene file and images will be
   deleted.  To keep the generated scene files or server response images for
   inspection purposes, set `cleanup=false` instead.  During the construction
   process a number of copies and clones are created, when `cleanup=false` there
   will be more than one empty temporary directory created that will not be
   deleted.  The path to the temporary directory can be observed by setting
   RenderEngineGltfClientParams::verbose to `true`, or inspecting the parent
   directory described by drake::temp_directory(). */
  bool cleanup = true;
=======
   drake::set_log_level() to `"debug"`.  @sa drake/common/text_logging.h */
  bool verbose = false;

  /** Whether or not the client should cleanup files generated / retrieved from
   the server.  By default (`no_cleanup=false`), as soon as a glTF scene file
   as well as server image response for a given frame are no longer needed they
   will be deleted.  To inspect generated scene files or server response images
   set `no_cleanup=true` to prevent the files and their governing temporary
   directory from being deleted.  During the construction process a number of
   copies and clones are created, when `no_cleanup=true` there will be more than
   one empty temporary directory created that will not be deleted.  The path
   to the temporary directory can be observed by setting
   RenderEngineGltfClientParams::verbose to `true`, or inspecting the parent
   directory described by drake::temp_directory(). */
  bool no_cleanup = false;
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

  /** Returns the post-processed full url used for client-server communication.
   The full url is constructed as `{base_url}/{render_endpoint}` where all
   trailing slashes in `base_url` and all leading slashes in `render_endpoint`
   have been removed. */
  std::string GetUrl() const;
};

}  // namespace geometry
}  // namespace drake
