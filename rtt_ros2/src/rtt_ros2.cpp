// Copyright 2020 Intermodalics BVBA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rtt_ros2/rtt_ros2.hpp"

#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "boost/filesystem.hpp"
#include "boost/version.hpp"

// TODO(meyerj): Replace libxml2 by tinyxml or xerces, which is already used by
// other ROS packages and/or RTT.
#include "libxml/parser.h"
#include "libxml/tree.h"
#include "libxml/xpath.h"
#include "libxml/xpathInternals.h"

#include "rtt/Logger.hpp"
#include "rtt/RTT.hpp"
#include "rtt/deployment/ComponentLoader.hpp"
#include "rtt/internal/GlobalService.hpp"

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"

namespace rtt_ros2
{

/// Helpers for libxml2 memory management using unique_ptr
namespace
{

/// Custom deleter for libxml2 types
struct XmlDeleter
{
public:
  void operator()(xmlChar * ptr) const {xmlFree(ptr);}
  void operator()(xmlDoc * ptr) const {xmlFreeDoc(ptr);}
  void operator()(xmlXPathContext * ptr) const {xmlXPathFreeContext(ptr);}
  void operator()(xmlXPathObject * ptr) const {xmlXPathFreeObject(ptr);}
};

typedef std::unique_ptr<xmlChar, XmlDeleter> xmlCharUniquePtr;
typedef std::unique_ptr<xmlDoc, XmlDeleter> xmlDocUniquePtr;
typedef std::unique_ptr<xmlXPathContext, XmlDeleter> xmlXPathContextUniquePtr;
typedef std::unique_ptr<xmlXPathObject, XmlDeleter> xmlXPathObjectUniquePtr;

}  // namespace

std::string find(const std::string & package)
{
  RTT::Logger::In in("rtt_ros2::find(\"" + package + "\")");

  try {
    return ament_index_cpp::get_package_share_directory(package);
  } catch (ament_index_cpp::PackageNotFoundError & e) {
    RTT::log(RTT::Error) << "Could not find ROS package '" << package <<
      "' in the ament index: " << e.what() <<
      RTT::endlog();
  }
  return {};
}

bool import(const std::string & package)
{
  RTT::Logger::In in("rtt_ros2::import(\"" + package + "\")");

  boost::shared_ptr<RTT::ComponentLoader> loader =
    RTT::ComponentLoader::Instance();

  // List of packages which could not be loaded
  std::set<std::string> missing_packages;

  // Add all rtt_ros2/plugin_depend dependencies to package names
  std::vector<std::string> deps_to_import;
  std::vector<std::string> search_paths;

  // Read the package.xml for this package and recursively for all its
  // plugin_depend dependencies
  std::queue<std::string> dep_names;
  dep_names.push(package);

  while (!dep_names.empty()) {
    namespace fs = boost::filesystem;

    // Get the next dep name
    const std::string dep_name = dep_names.front();
    dep_names.pop();

    // Find RTT plugin dependencies using the ament index
    {
      std::string plugin_depends;
      std::string prefix_path;
      if (ament_index_cpp::get_resource("rtt_ros2_plugin_depends", dep_name,
        plugin_depends, &prefix_path))
      {
        // The package was found. Try to import it...
        deps_to_import.push_back(dep_name);
        search_paths.push_back(
          (fs::path(prefix_path) / "lib" / "orocos").string());

        RTT::log(RTT::Debug) <<
          "Found package '" << dep_name <<
          "' in the ament index with the following dependencies:" <<
          RTT::endlog();

        // Decompose plugin_depends
        std::istringstream iss(plugin_depends);
        std::string plugin_depend;
        while (std::getline(iss, plugin_depend, ';')) {
          RTT::log(RTT::Debug) << " - " << plugin_depend << RTT::endlog();
          dep_names.push(plugin_depend);
        }

        // Do not parse package.xml anymore if the package has been found in the
        // ament index already
        continue;
      }
    }

    // Find path to package.xml and add the package's prefix path to the
    // RTT search path (DEPRECATED)
    constexpr bool kSearchPluginDependInPackageXml = true;
    if (kSearchPluginDependInPackageXml) {
      fs::path package_xml_path;
      try {
        package_xml_path =
          fs::path(ament_index_cpp::get_package_share_directory(dep_name)) /
          "package.xml";
      } catch (ament_index_cpp::PackageNotFoundError & e) {
        RTT::log(RTT::Error) << "Could not find ROS package '" << dep_name <<
          "' in the ament index: " << e.what() <<
          RTT::endlog();
        missing_packages.insert(dep_name);
        continue;
      }

      // Check if package.xml file exists
      if (!fs::is_regular_file(package_xml_path)) {
        RTT::log(RTT::Error) << "No package.xml file for ROS package '" <<
          dep_name << "' found at " <<
          package_xml_path.parent_path() << RTT::endlog();
        missing_packages.insert(dep_name);
        continue;
      }

      // The package was found. Try to import it...
      deps_to_import.push_back(dep_name);
      search_paths.push_back(
        (fs::path(ament_index_cpp::get_package_prefix(dep_name)) /
        "lib" / "orocos").string());

      // Load package.xml
      xmlInitParser();
      auto package_doc = xmlDocUniquePtr(xmlParseFile(package_xml_path.c_str()));
      auto xpath_ctx =
        xmlXPathContextUniquePtr(xmlXPathNewContext(package_doc.get()));
      static const auto rtt_plugin_depend_xpath = xmlCharUniquePtr(
        xmlCharStrdup("/package/export/rtt_ros2/plugin_depend/text()"));

      // Get the text of the rtt_ros2 <plugin_depend>s
      auto xpath_obj = xmlXPathObjectUniquePtr(
        xmlXPathEvalExpression(rtt_plugin_depend_xpath.get(), xpath_ctx.get()));

      // Iterate through the nodes
      if (xmlXPathNodeSetIsEmpty(xpath_obj->nodesetval)) {
        RTT::log(RTT::Debug) << "ROS package '" << dep_name <<
          "' has no RTT plugin dependencies." <<
          RTT::endlog();
      } else {
        RTT::log(RTT::Warning) <<
          "Listing RTT plugin dependencies in the <plugin_depend> tag in " <<
          dep_name << "/package.xml is depracted in rtt_ros2 and only "
          "supported for backwards compatibility." << RTT::nlog() <<
          "Instead, you can use the CMake macro "
          "rtt_ros2_export_plugin_depend(<dep>) to declare dependencies "
          "in " << dep_name << "/CMakeLists.txt." << RTT::endlog();

        RTT::log(RTT::Debug) <<
          "ROS package '" << dep_name << "' has " <<
          xpath_obj->nodesetval->nodeNr << " RTT plugin dependencies." <<
          RTT::endlog();

        for (int i = 0; i < xpath_obj->nodesetval->nodeNr; i++) {
          if (xpath_obj->nodesetval->nodeTab[i]) {
            std::ostringstream oss;
            auto dep_str = xmlCharUniquePtr(
              xmlNodeGetContent(xpath_obj->nodesetval->nodeTab[i]));
            oss << dep_str.get();
            dep_str.reset();

            RTT::log(RTT::Debug) <<
              "Found dependency '" << oss.str() << "' (from package.xml)" << RTT::endlog();
            dep_names.push(oss.str());
          }
        }
      }

      xmlCleanupParser();
    }
  }

  // Import packages in reverse order
  if (!deps_to_import.empty()) {
    // Build path list by prepending paths from search_paths list to the RTT
    // component path in reverse order without duplicates
    std::set<std::string> search_paths_seen;
    std::string path_list = loader->getComponentPath();
    for (auto it = search_paths.rbegin(); it != search_paths.rend(); ++it) {
      if (search_paths_seen.count(*it)) {continue;}
      path_list = *it + ":" + path_list;
      search_paths_seen.insert(*it);
    }

    RTT::log(RTT::Debug) << "Attempting to load RTT plugins from " <<
      deps_to_import.size() << " packages..." <<
      RTT::endlog();

    // Import each package dependency and the package itself
    // (in deps_to_import[0])
    for (auto it = deps_to_import.rbegin(); it != deps_to_import.rend(); ++it) {
      // Check if it's already been imported
      if (*it == "rtt_ros2" || loader->isImported(*it)) {
        RTT::log(RTT::Debug) << "Package dependency '" << *it <<
          "' already imported." << RTT::endlog();
        continue;
      }

      // Import the dependency
      if (loader->import(*it, path_list)) {
        RTT::log(RTT::Debug) << "Importing Orocos components from ROS package '" <<
          *it << "' SUCCEEDED.";
      } else {
        // Temporarily store the name of the missing package
        missing_packages.insert(*it);
        RTT::log(RTT::Debug) << "Importing Orocos components from ROS package '" <<
          *it << "' FAILED.";
      }
      RTT::log(RTT::Debug) << RTT::endlog();
    }
  }

  // Report success or failure
  if (missing_packages.size() == 0) {
    RTT::log(RTT::Info) << "Loaded plugins from ROS package '" << package <<
      "' and its dependencies." << RTT::endlog();
  } else {
    RTT::log(RTT::Warning) <<
      "Could not load RTT plugins from the following ROS packages:" <<
      RTT::endlog();
    for (const auto & pkg : missing_packages) {
      RTT::log(RTT::Warning) << " - " << pkg << RTT::endlog();
    }
  }

  return missing_packages.size() == 0;
}

}  // namespace rtt_ros2
