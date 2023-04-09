#include <cstring>

#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
<<<<<<< HEAD
#include "drake/bindings/pydrake/common/serialize_pybind.h"
=======
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  constexpr auto& doc = pydrake_doc.drake.lcm;

  py::module::import("pydrake.common");

  // Use `py::bytes` as a mid-point between C++ LCM (`void* + int` /
  // `vector<uint8_t>`) and Python LCM (`str`).
  using PyHandlerFunction = std::function<void(py::bytes)>;
  using PyMultichannelHandlerFunction =
      std::function<void(std::string_view, py::bytes)>;

  {
    using Class = DrakeLcmInterface;
    constexpr auto& cls_doc = doc.DrakeLcmInterface;
    py::class_<Class>(m, "DrakeLcmInterface", cls_doc.doc)
        .def("get_lcm_url", &DrakeLcmInterface::get_lcm_url,
            cls_doc.get_lcm_url.doc)
        .def(
            "Publish",
            [](Class* self, const std::string& channel, py::bytes buffer,
                std::optional<double> time_sec) {
              // TODO(eric.cousineau): See if there is a way to extra the raw
              // bytes from `buffer` without copying.
              std::string str = buffer;
              self->Publish(channel, str.data(), str.size(), time_sec);
            },
            py::arg("channel"), py::arg("buffer"),
            py::arg("time_sec") = py::none(), cls_doc.Publish.doc)
<<<<<<< HEAD
=======
        .def("HandleSubscriptions", &DrakeLcmInterface::HandleSubscriptions,
            py::arg("timeout_millis"), cls_doc.HandleSubscriptions.doc);
  }

  {
    using Class = DrakeLcmParams;
    constexpr auto& cls_doc = doc.DrakeLcmParams;
    py::class_<Class>(m, "DrakeLcmParams", cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("lcm_url", &Class::lcm_url, cls_doc.lcm_url.doc)
        .def_readwrite("channel_suffix", &Class::channel_suffix,
            cls_doc.channel_suffix.doc)
        .def_readwrite("defer_initialization", &Class::defer_initialization,
            cls_doc.defer_initialization.doc)
        .def("__repr__", [](const Class& self) {
          return py::str(
              "DrakeLcmParams("
              "lcm_url={}, "
              "channel_suffix={}, "
              "defer_initialization={})")
              .format(
                  self.lcm_url, self.channel_suffix, self.defer_initialization);
        });
  }

  {
    using Class = DrakeLcm;
    constexpr auto& cls_doc = doc.DrakeLcm;
    py::class_<Class, DrakeLcmInterface> cls(m, "DrakeLcm", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::string>(), py::arg("lcm_url"),
            cls_doc.ctor.doc_1args_lcm_url)
        .def(py::init<DrakeLcmParams>(), py::arg("params"),
            cls_doc.ctor.doc_1args_params)
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        .def(
            "Subscribe",
            [](Class* self, const std::string& channel,
                PyHandlerFunction handler) {
              auto subscription = self->Subscribe(
                  channel, [handler](const void* data, int size) {
                    handler(py::bytes(static_cast<const char*>(data), size));
                  });
              DRAKE_DEMAND(subscription != nullptr);
              // This is already the default, but for clarity we'll repeat it.
              subscription->set_unsubscribe_on_delete(false);
            },
<<<<<<< HEAD
            py::arg("channel"), py::arg("handler"), cls_doc.Subscribe.doc)
        .def(
            "SubscribeMultichannel",
            [](Class* self, const std::string& regex,
                PyMultichannelHandlerFunction handler) {
              auto subscription = self->SubscribeMultichannel(
                  regex, [handler](std::string_view channel, const void* data,
                             int size) {
                    handler(channel,
                        py::bytes(static_cast<const char*>(data), size));
                  });
              DRAKE_DEMAND(subscription != nullptr);
              // This is already the default, but for clarity we'll repeat it.
              subscription->set_unsubscribe_on_delete(false);
            },
            py::arg("regex"), py::arg("handler"),
            cls_doc.SubscribeMultichannel.doc)
        .def(
            "SubscribeAllChannels",
            [](Class* self, PyMultichannelHandlerFunction handler) {
              auto subscription =
                  self->SubscribeAllChannels([handler](std::string_view channel,
                                                 const void* data, int size) {
                    handler(channel,
                        py::bytes(static_cast<const char*>(data), size));
                  });
              DRAKE_DEMAND(subscription != nullptr);
              // This is already the default, but for clarity we'll repeat it.
              subscription->set_unsubscribe_on_delete(false);
            },
            py::arg("handler"), cls_doc.SubscribeAllChannels.doc)
        .def("HandleSubscriptions", &DrakeLcmInterface::HandleSubscriptions,
            py::arg("timeout_millis"), cls_doc.HandleSubscriptions.doc);
  }

  {
    using Class = DrakeLcmParams;
    constexpr auto& cls_doc = doc.DrakeLcmParams;
    py::class_<Class> cls(m, "DrakeLcmParams", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = DrakeLcm;
    constexpr auto& cls_doc = doc.DrakeLcm;
    py::class_<Class, DrakeLcmInterface> cls(m, "DrakeLcm", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::string>(), py::arg("lcm_url"),
            cls_doc.ctor.doc_1args_lcm_url)
        .def(py::init<DrakeLcmParams>(), py::arg("params"),
            cls_doc.ctor.doc_1args_params);
=======
            py::arg("channel"), py::arg("handler"), cls_doc.Subscribe.doc);
    // TODO(eric.cousineau): Add remaining methods.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const char* const doc_deprecated =
        cls_doc.ctor
            .doc_deprecated_deprecated_2args_lcm_url_defer_initialization;
    cls.def(py_init_deprecated<Class, std::string, bool>(doc_deprecated),
        py::arg("lcm_url"), py::arg("defer_initialization"), doc_deprecated);
#pragma GCC diagnostic pop
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
  }

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
