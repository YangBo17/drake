# A re-implementation of upstream's sdf/embedSdf.rb tool in Python.

import sys

<<<<<<< HEAD
import xml.etree.ElementTree as ET
=======
from lxml import etree
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c

assert __name__ == '__main__'


def _minified_xml(*, filename):
    """Given a filename for an `*.sdf` schema file, returns a minified xml
    string with its contents, to conserve disk space in Drake's library.
    """
<<<<<<< HEAD
    tree = ET.parse(filename)
    # Remove all '<description>' elements.
    for item in tree.findall(".//description/.."):
        item.remove(item.find("description"))
    # Discard whitespace.
    for elem in tree.iter('*'):
=======
    parser = etree.XMLParser(remove_blank_text=True, remove_comments=True)
    root = etree.parse(filename, parser)
    for item in root.xpath("//description"):
        item.getparent().remove(item)
    for elem in root.iter('*'):
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
        if elem.text is not None:
            elem.text = elem.text.strip()
        if elem.tail is not None:
            elem.tail = elem.tail.strip()
<<<<<<< HEAD
    return ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=False)
=======
    return etree.tostring(root, encoding="utf-8", xml_declaration=False)
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c


filenames = sorted(sys.argv[1:])
print("""
#include "EmbeddedSdf.hh"
#include <array>
<<<<<<< HEAD
#include "drake_vendor/gz/utils/NeverDestroyed.hh"
=======
#include "drake_vendor/ignition/utils/NeverDestroyed.hh"
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
namespace sdf { inline namespace SDF_VERSION_NAMESPACE {
const std::map<std::string, std::string>& GetEmbeddedSdf() {
  using Result = std::map<std::string, std::string>;
""")
print('  constexpr std::array<std::pair<const char*, const char*>, {}> pairs{{'
      .format(len(filenames)))
for filename in filenames:
    _, relative_path = filename.split('/sdf/')
    print('std::pair<const char*, const char*>{')
    print(f'"{relative_path}",')
    print('R"raw(')
    sys.stdout.flush()
    sys.stdout.buffer.write(_minified_xml(filename=filename))
    print(')raw"')
    print('},')
print("""
  };
<<<<<<< HEAD
  static const gz::utils::NeverDestroyed<Result> result{
=======
  static const ignition::utils::NeverDestroyed<Result> result{
>>>>>>> 39291320815eca6c872c9ce0a595d643d0acf87c
      pairs.begin(), pairs.end()};
  return result.Access();
}}}
""")
