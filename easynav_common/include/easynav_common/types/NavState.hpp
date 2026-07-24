// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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
/// \brief A blackboard-like structure to hold the current state of the navigation system.
///
/// This file defines the NavState class, which provides a thread-safe key-value store
/// where values can be of any type and stored/retrieved via smart pointers.
/// It is designed for concurrent, type-safe access in robotics applications.

#ifndef EASYNAV__TYPES__NAVSTATE_HPP_
#define EASYNAV__TYPES__NAVSTATE_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <type_traits>
#include <iostream>
#include <functional>
#include <execinfo.h>
#include <typeinfo>
#include <cxxabi.h>
#include <execinfo.h>

namespace easynav
{

/// \brief Demangles a C++ RTTI type name if possible.
/// \param name Mangled type name (from \c typeid(T).name()).
/// \return Readable (demangled) type name if available; otherwise returns \p name.
inline std::string demangle(const char * name)
{
  int status = 0;
  char * p = abi::__cxa_demangle(name, nullptr, nullptr, &status);
  std::string out = (status == 0 && p) ? p : name;
  std::free(p);
  return out;
}

/// \brief Captures a C/C++ stack trace as a string.
/// \param skip Number of initial frames to skip (e.g., the \c stacktrace frame itself).
/// \param max_frames Maximum number of frames to capture.
/// \return A multi-line string with one frame per line.
inline std::string stacktrace(std::size_t skip = 1, std::size_t max_frames = 64)
{
  void * buf[128];
  const int n = backtrace(buf, static_cast<int>(std::min<std::size_t>(max_frames, 128)));
  char ** syms = backtrace_symbols(buf, n);
  std::ostringstream oss;
  for (int i = static_cast<int>(skip); i < n; ++i) {
    oss << "#" << (i - static_cast<int>(skip)) << " " << syms[i] << "\n";
  }
  std::free(syms);
  return oss.str();
}

/// \class NavState
/// \brief A generic, type-safe, thread-safe blackboard to hold runtime state.
///
/// NavState provides:
/// - Type-erased storage using \c std::shared_ptr<void>.
/// - Runtime type verification and safe casting via \c typeid.
/// - Support for value-based and shared-pointer-based insertion.
/// - Debug utilities including stack trace and introspection.
///
/// \note Thread-safety is enforced with an internal \c std::mutex.
class NavState
{
public:
  /// \brief Constructs an empty NavState and registers basic type printers.
  NavState()
  {
    register_basic_printers();
  }

  /// \brief Destructor.
  virtual ~NavState() = default;

  /// \brief Stores a value of type \p T associated with \p key (by copy).
  ///
  /// If \p key does not exist, a new \c std::shared_ptr<T> is created and stored.
  /// If \p key exists, the stored value is overwritten in place.
  ///
  /// \tparam T Value type. Must be copy-assignable.
  /// \param key Key associated with the value.
  /// \param value Value to store (copied into internal storage).
  /// \throws std::runtime_error If \p key exists with a different stored type; includes a stack trace.
  template<typename T>
  void set(const std::string & key, const T & value)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      values_[key] = std::make_shared<T>(value);
      types_[key] = typeid(T).hash_code();
      type_names_[key] = demangle(typeid(T).name());
    } else {
      if (types_[key] != typeid(T).hash_code()) {
        std::ostringstream oss;
        oss << "Type mismatch in set(\"" << key << "\")\n"
            << "  stored type  : " << type_names_[key] << "\n"
            << "  provided type: " << demangle(typeid(T).name()) << "\n"
            << "Backtrace:\n" << stacktrace(1);
        throw std::runtime_error(oss.str());
      }

      auto ptr = std::static_pointer_cast<T>(it->second);
      *ptr = value;
    }
  }

  /// \brief Stores a value of type \p T associated with \p key (by shared pointer).
  ///
  /// If \p key does not exist, \p value_ptr is stored directly.
  /// If \p key exists, the stored \c std::shared_ptr<T> is replaced by \p value_ptr.
  ///
  /// \tparam T Value type.
  /// \param key Key associated with the value.
  /// \param value_ptr Shared pointer to the value to store.
  /// \throws std::runtime_error If \p key exists with a different stored type; includes a stack trace.
  template<typename T>
  void set(const std::string & key, const std::shared_ptr<T> value_ptr)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      values_[key] = std::shared_ptr<T>(value_ptr);
      types_[key] = typeid(T).hash_code();
      type_names_[key] = demangle(typeid(T).name());
    } else {
      if (types_[key] != typeid(T).hash_code()) {
        std::ostringstream oss;
        oss << "Type mismatch in set(\"" << key << "\")\n"
            << "  stored type  : " << type_names_[key] << "\n"
            << "  provided type: " << demangle(typeid(T).name()) << "\n"
            << "Backtrace:\n" << stacktrace(1);
        throw std::runtime_error(oss.str());
      }

      it->second = value_ptr;
    }
  }

  /// \brief Sets a new group of values as a list of strings. It aslo stores the group keys as a value for introspection/debugging purposes.
  /// The group consists of a vector of keys, where each key points to a NavState element.
  ///
  /// If \p key does not exist, a new list is created and stored.
  /// If \p key exists, the stored list is overwritten in place.
  ///
  /// \param key Key associated with the group.
  /// \param group_keys Value to store. The list of other keys in the NavState that compose this group.
  void set_group(const std::string & key, const std::vector<std::string> & group_keys)
  {
    std::lock_guard<std::mutex> lock(group_mutex_);
    groups_[key] = group_keys;

    // Also store the group keys as a value for introspection/debugging purposes.
    set<std::vector<std::string>>(key, group_keys);
  }

  /// \brief Retrieves a const reference to the stored value of type \p T for \p key.
  ///
  /// The reference refers to the object managed by the internal \c std::shared_ptr<T>.
  ///
  /// \tparam T Expected stored type.
  /// \param key Key to retrieve.
  /// \return Const reference to the stored \p T.
  /// \throws std::runtime_error If \p key is missing or the stored type does not match \p T.
  template<typename T>
  const T & get(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      throw std::runtime_error("Key not found in get: " + key);
    }

    if (types_.at(key) != typeid(T).hash_code()) {
      std::ostringstream oss;
      oss << "Type mismatch in get(\"" << key << "\")\n"
          << "  stored type   : " << type_names_.at(key) << "\n"
          << "  requested type: " << demangle(typeid(T).name());
      throw std::runtime_error(oss.str());
    }

    auto ptr = std::static_pointer_cast<T>(it->second);
    return *ptr;
  }

  /// \brief Retrieves the shared_ptr to the stored value of type \p T for \p key.
  ///
  /// The pointer refers to the object managed by the internal \c std::shared_ptr<T>.
  ///
  /// \tparam T Expected stored type.
  /// \param key Key to retrieve.
  /// \return shared_ptr to the stored \p T.
  /// \throws std::runtime_error If \p key is missing or the stored type does not match \p T.
  template<typename T>
  const std::shared_ptr<T> get_ptr(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      throw std::runtime_error("Key not found in get: " + key);
    }

    if (types_.at(key) != typeid(T).hash_code()) {
      std::ostringstream oss;
      oss << "Type mismatch in get_ptr(\"" << key << "\")\n"
          << "  stored type   : " << type_names_.at(key) << "\n"
          << "  requested type: " << demangle(typeid(T).name());
      throw std::runtime_error(oss.str());
    }

    return std::static_pointer_cast<T>(it->second);
  }

  /// \brief Retrieves a vector of values (pointers) from a group by the group key.
  ///
  /// The value pointers refer to the object managed by the internal \c std::shared_ptr<T>.
  ///
  /// \tparam T Expected stored type.
  /// \param group_key Group key to retrieve.
  /// \return vector of shared_ptr to the stored \p T.
  /// \throws std::runtime_error If any \p key is missing or the stored type does not match \p T.
  template<typename T>
  const std::vector<std::shared_ptr<T>> get_group(const std::string & group_key) const
  {
    std::lock_guard<std::mutex> lock(group_mutex_);
    auto group_it = groups_.find(group_key);

    if (group_it == groups_.end()) {
      throw std::runtime_error("Group key not found in get_group: " + group_key);
    }

    const auto group_keys = group_it->second;

    std::vector<std::shared_ptr<T>> out;
    out.reserve(group_keys.size());

    for (const auto & group_key : group_keys) {
      try {
        out.push_back(get_ptr<T>(group_key));
      } catch (const std::runtime_error & e) {
        std::cerr << "Error retrieving key '" << group_key << "' from group '" << group_key <<
          "': " << e.what() << std::endl;
      }
    }

    return out;
  }

  /// \brief Retrieves all stored values of type \p T, regardless of their key.
  ///
  /// Scans every entry in the state and returns those whose stored type matches \p T.
  /// Entries belonging to group-metadata keys (stored as \c std::vector<std::string>)
  /// are automatically excluded because their type hash will not match \p T.
  ///
  /// \tparam T Expected stored type.
  /// \return Vector of shared_ptr to every stored \p T. Empty if none are found.
  template<typename T>
  std::vector<std::shared_ptr<T>> get_by_type() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<std::shared_ptr<T>> out;
    const size_t target_hash = typeid(T).hash_code();
    for (const auto & kv : values_) {
      if (types_.at(kv.first) == target_hash) {
        out.push_back(std::static_pointer_cast<T>(kv.second));
      }
    }
    return out;
  }

  /// \brief Wraps a single stored value of type \p T in a one-element vector.
  ///
  /// Convenience to allow uniform code that always works with
  /// \c std::vector<std::shared_ptr<T>> regardless of whether the caller
  /// has one sensor or many (via \c get_group).
  /// - If \p key is not found or the stored type does not match \p T, returns an empty vector.
  ///
  /// \tparam T Expected stored type.
  /// \param key Key of the individual value to retrieve.
  /// \return A one-element vector with the shared_ptr to \p T, or empty on miss/mismatch.
  template<typename T>
  std::vector<std::shared_ptr<T>> get_to_vector(const std::string & key) const
  {
    std::vector<std::shared_ptr<T>> out;
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = values_.find(key);
    if (it == values_.end()) {
      return out;
    }
    if (types_.at(key) != typeid(T).hash_code()) {
      return out;
    }
    out.push_back(std::static_pointer_cast<T>(it->second));
    return out;
  }

  /// \brief Retrieves all stored values of type \p T that are NOT a member of any group.
  ///
  /// This is the complement of \c get_group: it returns every entry whose type matches \p T
  /// and whose key does not appear in any group's member list.
  /// Group-metadata keys (the \c std::vector<std::string> stored by \c set_group) are excluded
  /// by type, so they will never appear in results unless \p T is \c std::vector<std::string>.
  ///
  /// \tparam T Expected stored type.
  /// \return Vector of shared_ptr to every ungrouped \p T. Empty if none are found.
  template<typename T>
  std::vector<std::shared_ptr<T>> get_no_group() const
  {
    // Snapshot the set of all keys that belong to any group (under group lock).
    std::unordered_set<std::string> grouped_keys;
    {
      std::lock_guard<std::mutex> glock(group_mutex_);
      for (const auto & g : groups_) {
        for (const auto & k : g.second) {
          grouped_keys.insert(k);
        }
      }
    }

    // Iterate values and return those of type T whose key is not grouped.
    std::lock_guard<std::mutex> slock(state_mutex_);
    std::vector<std::shared_ptr<T>> out;
    const size_t target_hash = typeid(T).hash_code();
    for (const auto & kv : values_) {
      if (types_.at(kv.first) == target_hash &&
        grouped_keys.find(kv.first) == grouped_keys.end())
      {
        out.push_back(std::static_pointer_cast<T>(kv.second));
      }
    }
    return out;
  }

  /// \brief Checks whether \p key exists in the state.
  /// \param key Key to query.
  /// \return \c true if present, otherwise \c false.
  bool has(const std::string & key) const
  {
    return values_.find(key) != values_.end();
  }

  /// \brief Checks whether \p key exists in the state.
  /// \param key Key to query.
  /// \return \c true if present, otherwise \c false.
  bool has_group(const std::string & key) const
  {
    return groups_.find(key) != groups_.end();
  }

  /// \brief Type alias for a generic printer functor used by \ref debug_string().
  ///
  /// The functor receives the stored value as a \c std::shared_ptr<void>
  /// and returns a string representation.
  using AnyPrinter = std::function<std::string(std::shared_ptr<void>)>;

  /// \brief Registers a pretty-printer for type \p T used by \ref debug_string().
  ///
  /// \tparam T Type to register.
  /// \param printer Functor that renders \c const T& to a string.
  template<typename T>
  static void register_printer(std::function<std::string(const T &)> printer)
  {
    auto wrapper = [printer](std::shared_ptr<void> base_ptr) -> std::string {
        auto typed_ptr = std::static_pointer_cast<T>(base_ptr);
        return printer(*typed_ptr);
      };
    type_printers_[typeid(T).hash_code()] = wrapper;
  }

  /// \brief Generates a human-readable dump of all stored keys and values.
  ///
  /// For types with registered printers, their printer is used; otherwise, the raw pointer
  /// address and type hash are shown.
  /// \return Multi-line string with one entry per key.
  std::string debug_string() const
  {
    std::stringstream ss;
    for (const auto & kv : values_) {
      ss << kv.first << " = ";
      auto ptr = kv.second;
      if (ptr) {
        auto type_it = types_.find(kv.first);
        if (type_it != types_.end()) {
          auto printer_it = type_printers_.find(type_it->second);
          if (printer_it != type_printers_.end()) {
            ss << "[" << ptr.get() << "] : " << printer_it->second(ptr);
          } else {
            ss << "[" << ptr.get() << "] : " << type_it->second << "]";
          }
        } else {
          ss << "[" << ptr.get() << "] : unknown]";
        }
      } else {
        ss << "[null]";
      }
      ss << std::endl;
    }
    return ss.str();
  }

  /// \brief Prints the current C++ stack trace to \c std::cerr.
  /// \note Intended for debugging in exception contexts.
  static void print_stacktrace()
  {
    void *array[50];
    int size = backtrace(array, 50);
    char **strings = backtrace_symbols(array, size);
    std::cerr << "\nStack trace:\n";
    for (int i = 0; i < size; ++i) {
      std::cerr << strings[i] << std::endl;
    }
    std::cerr << std::endl;
    free(strings);
  }

  /// \brief Registers default printers for common scalar types.
  ///
  /// Registers printers for: \c int, \c float, \c double, \c std::string, \c bool, \c char.
  static void register_basic_printers()
  {
    register_printer<int>([](const int & v) {return std::to_string(v);});
    register_printer<float>([](const float & v) {return std::to_string(v);});
    register_printer<double>([](const double & v) {return std::to_string(v);});
    register_printer<std::string>([](const std::string & v) {return v;});
    register_printer<bool>([](const bool & v) {return v ? "true" : "false";});
    register_printer<char>([](const char & v) {return std::string(1, v);});
    register_printer<std::vector<std::string>>(
      [](const std::vector<std::string> & v) {
        std::ostringstream oss;
        oss << " " << v.size() << " [";
        for (std::size_t i = 0; i < v.size(); ++i) {
          if (i > 0) {oss << ", ";}
          oss << v[i];
        }
        oss << "]";
        return oss.str();
      });
  }

private:
  mutable std::mutex state_mutex_;  ///< Guards access to \ref values_ and \ref types_.
  mutable std::mutex group_mutex_;  ///< Guards access to \ref groups_.

  /// \brief Internal storage of values as type-erased shared pointers.
  mutable std::unordered_map<std::string, std::shared_ptr<void>> values_;

  /// \brief Group of NavState values, indexed by their keys. Useful to get multiple sensors together.
  mutable std::unordered_map<std::string, std::vector<std::string>> groups_;

  /// \brief Stored type hash (from \c typeid(T).hash_code()) per key.
  mutable std::unordered_map<std::string, size_t> types_;

  /// \brief Stored demangled type name per key, for error reporting.
  mutable std::unordered_map<std::string, std::string> type_names_;

  /// \brief Registry of type-hash -> printer functors used by \ref debug_string().
  static inline std::unordered_map<size_t, AnyPrinter> type_printers_;
};

}  // namespace easynav

#endif  // EASYNAV__TYPES__NAVSTATE_HPP_
