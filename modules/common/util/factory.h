/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Defines the Factory class.
 */

#pragma once

#include <map>
#include <memory>
#include <utility>

#include "cyber/common/macros.h"

#include "cyber/common/log.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

/**
 * @class Factory
 * @brief Implements a Factory design pattern with Register and Create methods
 *
 * The objects created by this factory all implement the same interface
 * (namely, AbstractProduct). This design pattern is useful in settings where
 * multiple implementations of an interface are available, and one wishes to
 * defer the choice of the implementation in use.
 *
 * @param IdentifierType Type used for identifying the registered classes,
 * typically std::string.接口的名字
 * @param AbstractProduct The interface implemented by the registered classes
 *                        要创建的接口
 * @param ProductCreator Function returning a pointer to an instance of
 * the registered class
 * @param MapContainer Internal implementation of the function mapping
 * IdentifierType to ProductCreator, by default std::unordered_map
 */
template <typename IdentifierType, class AbstractProduct,
          class ProductCreator = AbstractProduct *(*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
class Factory {
 public:
  /**
   * @brief Registers the class given by the creator function, linking it to id.
   * Registration must happen prior to calling CreateObject.
   * @param id Identifier of the class being registered
   * @param creator Function returning a pointer to an instance of
   * the registered class
   * @return True if the key id is still available
   * @note 对于std::map来说，其不允许重复的关键字，若插入一个已存在的关键字，则插入失败
   *       insert的返回值是一个std::pair，first代表插入元素的迭代器，second代表是否插入成功
   */
  bool Register(const IdentifierType &id, ProductCreator creator) {
    return producers_.insert(std::make_pair(id, creator)).second;
  }

  /**
   * @brief 检查该类型是否在关联容器当中
   * @note find函数查找的元素在容器中不存在，则返回一个尾后迭代器
   * 
   * @param id 要查找的类型
   * @return true 该类型在关联容器当中
   * @return false 该类型不在关联容器当中
   */
  bool Contains(const IdentifierType &id) {
    return producers_.find(id) != producers_.end();
  }

  /**
   * @brief Unregisters the class with the given identifier
   * @param id The identifier of the class to be unregistered
   */
  bool Unregister(const IdentifierType &id) {
    return producers_.erase(id) == 1;
  }

  void Clear() { producers_.clear(); }

  bool Empty() const { return producers_.empty(); }

  /**
   * @brief Creates and transfers membership of an object of type matching id.
   * Need to register id before CreateObject is called. May return nullptr
   * silently.
   * @param id The identifier of the class we which to instantiate
   * @param args the object construction arguments
   * @note - 对于map关联容器来讲，find返回值是mapped_pair，即一个std::pair对象
   *       - std::unique_ptr虽然不支持拷贝，但支持返回，此时会发生浅复制
   *       - 模板参数为右值引用，且转发使用std::forward，则会保持参数所有的属性，包括左值右值，const等
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType &id,
                                                      Args &&... args) {
    auto id_iter = producers_.find(id);
    if (id_iter != producers_.end()) {
      return std::unique_ptr<AbstractProduct>(
          (id_iter->second)(std::forward<Args>(args)...));
    }
    return nullptr;
  }
  
  /**
   * @brief Creates and transfers membership of an object of type matching id.
   * Need to register id before CreateObject is called.
   * @param id The identifier of the class we which to instantiate
   * @param args the object construction arguments
   * @note - 使用可变参数列表，接受可变数目和可变类型的参数
   *       - 使用类型参数的右值引用以及std::forward，可以转发参数所有的性质，包括左值右值，const等
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType &id,
                                                Args &&... args) {
    auto result = CreateObjectOrNull(id, std::forward<Args>(args)...);
    AERROR_IF(!result) << "Factory could not create Object of type : " << id;
    return result;
  }

 private:
  MapContainer producers_; ///< 保存该接口的名字和初始化该接口函数的关联容器
};

}  // namespace util
}  // namespace common
}  // namespace apollo
