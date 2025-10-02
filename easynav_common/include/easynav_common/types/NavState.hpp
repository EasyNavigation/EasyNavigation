// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief A blackboard-like structure to hold the current state of the navigation system.
///
/// This file defines the NavState class, which provides a lock-free key-value store
/// where values can be of any type and stored/retrieved via smart pointers.
/// It is designed for concurrent, type-safe access in robotics applications.

#ifndef EASYNAV__TYPES__NAVSTATE_HPP_
#define EASYNAV__TYPES__NAVSTATE_HPP_

#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <type_traits>
#include <iostream>
#include <functional>
#include <execinfo.h>
#include <typeinfo>

namespace easynav
{


/// \class NavState
/// \brief A generic, type-safe, lock-free blackboard to hold runtime state.
///
/// NavState provides:
/// - Type-erased storage using `std::shared_ptr<void>`.
/// - Runtime type verification and safe casting via `typeid`.
/// - Support for raw, shared, and copy-based insertion.
/// - Debug utilities including stack trace and introspection.
///
/// Example usage:
/// ```cpp
/// NavState state;
/// state.set("goal_reached", false);
/// bool reached = state.get<bool>("goal_reached");
/// ```
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

  /// \brief Stores a value of type T associated with the given key.
  ///
  /// If the key does not exist, a new shared_ptr<T> is created and stored.
  /// If the key already exists, the stored value is updated in-place.
  ///
  /// The value is internally managed through a shared_ptr<T>.
  ///
  /// \tparam T The type of the value to store. Must be copy-assignable.
  /// \param key The key associated with the value.
  /// \param value The value to store.
  /// \throws std::runtime_error if there is a type mismatch with an existing key.
  template<typename T>
  void set(const std::string & key, const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      values_[key] = std::make_shared<T>(value);
      types_[key] = typeid(T).hash_code();
    } else {
      if (types_[key] != typeid(T).hash_code()) {
        throw std::runtime_error("Type mismatch in set for key: " + key);
      }

      auto ptr = std::static_pointer_cast<T>(it->second);
      *ptr = value;
    }
  }

  /// \brief Stores a value of type T associated with the given key.
  ///
  /// If the key does not exist, the new shared_ptr<T> is stored.
  /// If the key already exists, the stored shared_ptr<T> is updated.
  ///
  /// The value is internally managed through a shared_ptr<T>.
  ///
  /// \tparam T The type of the value to store. Must be copy-assignable.
  /// \param key The key associated with the value.
  /// \param value_ptr A shared_ptr of the value to store.
  /// \throws std::runtime_error if there is a type mismatch with an existing key.
  template<typename T>
  void set(const std::string & key, const std::shared_ptr<T> value_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      values_[key] = std::shared_ptr<T>(value_ptr);
      types_[key] = typeid(T).hash_code();
    } else {
      if (types_[key] != typeid(T).hash_code()) {
        throw std::runtime_error("Type mismatch in set for key: " + key);
      }

      auto ptr = std::static_pointer_cast<T>(it->second);
      ptr = std::shared_ptr<T>(value_ptr);
    }
  }

  /// \brief Retrieves a const reference to the value of type T associated with the given key.
  ///
  /// The reference points to the value managed internally through a shared_ptr<T>.
  /// This avoids unnecessary copies, but care must be taken not to hold the reference
  /// beyond the lifetime of the NavState instance.
  ///
  /// \tparam T The expected type of the stored value.
  /// \param key The key of the value to retrieve.
  /// \return const T& A const reference to the stored value.
  /// \throws std::runtime_error if the key is not found or there is a type mismatch.
  template<typename T>
  const T & get(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);

    if (it == values_.end()) {
      throw std::runtime_error("Key not found in get: " + key);
    }

    if (types_.at(key) != typeid(T).hash_code()) {
      throw std::runtime_error("Type mismatch in get for key: " + key);
    }

    auto ptr = std::static_pointer_cast<T>(it->second);
    return *ptr;
  }

  /// \brief Checks whether a key exists in the NavState.
  /// \param key Lookup key.
  /// \return True if the key is registered.
  bool has(const std::string & key) const
  {
    return values_.find(key) != values_.end();
  }

  /// \brief Type alias for a generic printer function.
  ///
  /// Used to print debug output for stored values.
  using AnyPrinter = std::function<std::string(std::shared_ptr<void>)>;

  /// \brief Registers a printer for a given type.
  ///
  /// The function will be used to convert values of this type into strings
  /// for use in `debug_string()`.
  ///
  /// \tparam T Type to register.
  /// \param printer Function that converts a const reference to string.
  template<typename T>
  static void register_printer(std::function<std::string(const T &)> printer)
  {
    auto wrapper = [printer](std::shared_ptr<void> base_ptr) -> std::string {
        auto typed_ptr = std::static_pointer_cast<T>(base_ptr);
        return printer(*typed_ptr);
      };
    type_printers_[typeid(T).hash_code()] = wrapper;
  }

  /// \brief Dumps all keys and their values to a formatted string.
  ///
  /// If a printer is registered for a given type, it is used;
  /// otherwise, the raw pointer address and hash are shown.
  ///
  /// \return String representation of current state.
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

  /// \brief Prints the current C++ stack trace to standard error.
  ///
  /// Used to assist debugging in exception contexts.
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

  /// \brief Registers default string printers for basic types:
  /// `int`, `float`, `double`, `std::string`, `bool`, `char`.
  static void register_basic_printers()
  {
    register_printer<int>([](const int & v) {return std::to_string(v);});
    register_printer<float>([](const float & v) {return std::to_string(v);});
    register_printer<double>([](const double & v) {return std::to_string(v);});
    register_printer<std::string>([](const std::string & v) {return v;});
    register_printer<bool>([](const bool & v) {return v ? "true" : "false";});
    register_printer<char>([](const char & v) {return std::string(1, v);});
  }

private:
  mutable std::mutex mutex_;

  /// \brief Internal storage of values as shared void pointers.
  mutable std::unordered_map<std::string, std::shared_ptr<void>> values_;

  /// \brief Stores typeid hashes for each key.
  mutable std::unordered_map<std::string, size_t> types_;

  /// \brief Maps typeid hashes to printable string renderers.
  static inline std::unordered_map<size_t, AnyPrinter> type_printers_;
};

}  // namespace easynav

#endif  // EASYNAV__TYPES__NAVSTATE_HPP_
