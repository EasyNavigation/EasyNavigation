// Copyright 2025 Intelligent Robotics Lab
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

/// \file
/// \brief Lifecycle-aware plugin loading and runtime switching shared by the EasyNav nodes.

#ifndef EASYNAV_CORE__PLUGINSWITCHER_HPP_
#define EASYNAV_CORE__PLUGINSWITCHER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @class PluginSwitcher
 * @brief Owns the plugin instances of a lifecycle node and reloads them on every configure.
 *
 * The node declares a "types parameter" (for instance "controller_types") holding the aliases
 * of the plugins to load. Each alias X has a parameter "X.plugin" with the pluginlib class name
 * (it may also come only from the parameter overrides).
 *
 * Switching plugins is done with the regular lifecycle, without any parameter callback:
 * deactivate, cleanup, set the types parameter and configure again.
 *
 *  - \ref configure loads the plugins listed in the types parameter (call it from on_configure).
 *  - \ref release drops the instances. Call it from on_cleanup, on_shutdown, on_error and the
 *    destructor; it is idempotent. The plugin libraries are never unloaded (see
 *    keep_libraries_loaded()).
 *
 * Parameters stay declared in the node after a release (rclcpp cannot undeclare statically
 * typed parameters), with their last value. Plugins must therefore check has_parameter()
 * before declare_parameter() in on_initialize(), or they fail on the second configure. Values
 * for a plugin that is not loaded yet can be given as parameter overrides: they are used when
 * the plugin declares its parameters.
 *
 * Instances are handed out as shared_ptr copies (\ref get, \ref get_all), so a cycle running
 * in another thread keeps its plugin alive even if it is released concurrently.
 *
 * @tparam PluginT Plugin base class (derived from MethodBase).
 */
template<typename PluginT>
class PluginSwitcher
{
public:
  /**
   * @param node Lifecycle node owning the plugins. Must outlive this object.
   * @param package Package that exports the plugin descriptions.
   * @param base_class Fully qualified plugin base class name.
   * @param types_parameter Name of the parameter listing the plugin aliases.
   * @param max_plugins Maximum number of simultaneous plugins (0 means unlimited).
   */
  PluginSwitcher(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & package,
    const std::string & base_class,
    const std::string & types_parameter,
    size_t max_plugins = 1)
  : node_(node),
    loader_(std::make_unique<pluginlib::ClassLoader<PluginT>>(package, base_class)),
    base_class_(base_class),
    types_parameter_(types_parameter),
    max_plugins_(max_plugins)
  {
  }

  ~PluginSwitcher()
  {
    release();
    keep_libraries_loaded();
  }

  PluginSwitcher(const PluginSwitcher &) = delete;
  PluginSwitcher & operator=(const PluginSwitcher &) = delete;

  /**
   * @brief Loads the plugins listed in the types parameter, declaring it if needed.
   * @return false (and nothing loaded) if the configuration is wrong or a plugin fails.
   */
  bool configure()
  {
    std::vector<std::string> types;
    if (!node_.has_parameter(types_parameter_)) {
      node_.declare_parameter(types_parameter_, types);
    }
    node_.get_parameter(types_parameter_, types);

    if (max_plugins_ != 0 && types.size() > max_plugins_) {
      RCLCPP_ERROR(node_.get_logger(),
        "[%s] at most %lu plugin(s) can be instanced. [%lu] found",
        types_parameter_.c_str(), static_cast<unsigned long>(max_plugins_),
        static_cast<unsigned long>(types.size()));
      return false;
    }

    std::vector<Loaded> loaded;
    for (const auto & type : types) {
      auto item = create(type, resolve_plugin_class(type));
      if (!item) {
        discard(loaded);
        return false;
      }
      loaded.push_back(std::move(*item));
    }

    std::lock_guard<std::mutex> lock(plugins_mutex_);
    plugins_ = std::move(loaded);
    return true;
  }

  /**
   * @brief Drops every plugin instance (the libraries stay loaded). Idempotent.
   */
  void release()
  {
    std::vector<Loaded> old;
    {
      std::lock_guard<std::mutex> lock(plugins_mutex_);
      old = std::move(plugins_);
      plugins_.clear();
    }
    release_items(old);
  }

  /// @brief First loaded plugin, or nullptr. Safe to call from any thread.
  std::shared_ptr<PluginT> get() const
  {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    return plugins_.empty() ? nullptr : plugins_.front().instance;
  }

  /// @brief Copy of the loaded plugins. Safe to call from any thread.
  std::vector<std::shared_ptr<PluginT>> get_all() const
  {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    std::vector<std::shared_ptr<PluginT>> ret;
    for (const auto & item : plugins_) {
      ret.push_back(item.instance);
    }
    return ret;
  }

  /// @brief Aliases of the plugins that are really loaded.
  std::vector<std::string> loaded_types() const
  {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    return aliases_of(plugins_);
  }

private:
  struct Loaded
  {
    std::string alias;
    std::string plugin_class;
    std::shared_ptr<PluginT> instance;
  };

  static std::vector<std::string> aliases_of(const std::vector<Loaded> & items)
  {
    std::vector<std::string> ret;
    for (const auto & item : items) {
      ret.push_back(item.alias);
    }
    return ret;
  }

  /// Class name of an alias: its "<alias>.plugin" parameter, or else the parameter overrides.
  std::string resolve_plugin_class(const std::string & alias) const
  {
    const std::string name = alias + ".plugin";
    if (node_.has_parameter(name)) {
      return node_.get_parameter(name).as_string();
    }

    const auto & overrides = node_.get_node_parameters_interface()->get_parameter_overrides();
    const auto it = overrides.find(name);
    if (it != overrides.end() && it->second.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return it->second.get<std::string>();
    }
    return "";
  }

  std::optional<Loaded> create(const std::string & alias, const std::string & plugin_class)
  {
    try {
      RCLCPP_INFO(node_.get_logger(), "Loading %s %s [%s]",
        base_class_.c_str(), alias.c_str(), plugin_class.c_str());

      if (!node_.has_parameter(alias + ".plugin")) {
        node_.declare_parameter(alias + ".plugin", plugin_class);
      }

      Loaded item;
      item.alias = alias;
      item.plugin_class = plugin_class;
      item.instance = loader_->createSharedInstance(plugin_class);
      item.instance->initialize(node_.shared_from_this(), alias);

      RCLCPP_INFO(node_.get_logger(), "Loaded %s %s [%s]",
        base_class_.c_str(), alias.c_str(), plugin_class.c_str());
      return item;
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      RCLCPP_ERROR(node_.get_logger(),
        "Unable to load %s [%s] plugin [%s]: %s. The node keeps the parameters of a plugin "
        "declared after it is released (rclcpp cannot undeclare statically typed parameters), "
        "so the plugin must call has_parameter() before declare_parameter().",
        base_class_.c_str(), alias.c_str(), plugin_class.c_str(), e.what());
      return std::nullopt;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_.get_logger(), "Unable to load %s [%s] plugin [%s]. Error: %s",
        base_class_.c_str(), alias.c_str(), plugin_class.c_str(), e.what());
      return std::nullopt;
    }
  }

  /// Drops instances that were created but will not be used.
  void discard(std::vector<Loaded> & items)
  {
    release_items(items);
    items.clear();
  }

  /// Drops the instances. Their libraries stay loaded, see \ref keep_libraries_loaded.
  void release_items(std::vector<Loaded> & items)
  {
    for (auto & item : items) {
      item.instance.reset();
    }
  }

  /**
   * Hands the class loader over to a holder that is never destroyed, so the plugin libraries
   * are not unloaded while the nodes are created, released and destroyed.
   *
   * Plugins leave code of their library in global state: for instance the printers they
   * register with NavState::register_printer() are std::function objects in a process-wide
   * map, whose code lives in the plugin library. If that library is unloaded and another
   * plugin then registers a printer for the same type, replacing the old one calls code that is
   * gone. Loading a plugin right after unloading the library of another one also crashed
   * inside class_loader (seen with easynav_vff_controller after easynav_simple_controller).
   * Reloading the very same library usually hides the problem because it lands at the same
   * address. The libraries are still closed by class_loader when the process exits.
   */
  void keep_libraries_loaded()
  {
    static auto * const holder = new std::vector<std::shared_ptr<void>>();
    static std::mutex holder_mutex;

    std::lock_guard<std::mutex> lock(holder_mutex);
    holder->push_back(std::shared_ptr<void>(std::move(loader_)));
  }

  rclcpp_lifecycle::LifecycleNode & node_;

  // Declared before plugins_ so it is destroyed after them: the code of the instances
  // lives in the libraries this loader keeps open.
  std::unique_ptr<pluginlib::ClassLoader<PluginT>> loader_;
  const std::string base_class_;
  const std::string types_parameter_;
  const size_t max_plugins_;

  mutable std::mutex plugins_mutex_;
  std::vector<Loaded> plugins_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__PLUGINSWITCHER_HPP_
