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


#ifndef EASYNAV_COMMON__SINGLETON_H_
#define EASYNAV_COMMON__SINGLETON_H_

#include <memory>
#include <mutex>
#include <utility>

namespace easynav
{

template<class C>
class Singleton
{
public:
  template<typename ... Args>
  static C * getInstance(Args &&... args)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!instance_) {
      instance_ = std::make_unique<C>(std::forward<Args>(args)...);
    }
    return instance_.get();
  }

  /// \brief Destroy the current instance, if any; the next getInstance()/get() call
  /// creates a fresh one.
  ///
  /// \warning Only safe to call when no other thread may still be dereferencing a
  /// \c C* obtained from an earlier getInstance()/get() call -- that pointer's
  /// lifetime is tied to the destroyed instance, and no amount of locking here can
  /// protect a caller who is already holding and using it (the lock only serializes
  /// this class's own instance_/mutex_ bookkeeping against concurrent getInstance()/
  /// removeInstance() calls). Intended for sequential use, e.g. resetting singleton
  /// state between test cases, not for tearing down an instance while it may still
  /// be in use elsewhere.
  static void removeInstance()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    instance_.reset();
  }

  template<typename ... Args>
  static C & get(Args &&... args)
  {
    return *getInstance(std::forward<Args>(args)...);
  }

protected:
  Singleton() = default;
  ~Singleton() = default;

  Singleton(const Singleton &) = delete;
  Singleton & operator=(const Singleton &) = delete;

private:
  static std::unique_ptr<C> instance_;
  static std::mutex mutex_;
};

template<class C>
std::unique_ptr<C> Singleton<C>::instance_ = nullptr;

template<class C>
std::mutex Singleton<C>::mutex_;

#define SINGLETON_DEFINITIONS(ClassName) \
public: \
  static ClassName * getInstance() \
  { \
    return ::easynav::Singleton<ClassName>::getInstance(); \
  } \
  template<typename ... Args> \
  static ClassName * getInstance(Args && ... args) \
  { \
    return ::easynav::Singleton<ClassName>::getInstance( \
      std::forward<Args>(args)...); \
  }

}  // namespace easynav

#endif  // EASYNAV_COMMON__SINGLETON_H_
