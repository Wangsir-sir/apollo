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
 * @file message_manager.h
 * @brief The class of MessageManager
 */
#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/canbus/common/byte.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

using apollo::common::ErrorCode;
using apollo::cyber::Time;
using micros = std::chrono::microseconds;

/**
 * @struct CheckIdArg
 *
 * @brief this struct include data for check ids.
 */
struct CheckIdArg {
  int64_t period = 0; ///< 该协议的发送停顿间隔
  int64_t real_period = 0; ///< 该协议的can报文被解析的时间间隔
  int64_t last_time = 0; ///< 上次该协议的can报文被解析的时间
  int32_t error_count = 0; ///< 超时的次数
};

/**
 * @class MessageManager 协议管理类
 *
 * @brief message manager manages protocols. It supports parse and can get
 * protocol data by message id.
 * @details 是各个不同车型的协议管理类的基类，在其派生类当中，创建需要的协议类堆对象，
 *          注意其他代码当中需要用到协议类时，都必须通过指向在此创建的堆对象的指针来调用协议类，
 *          含有保存发送接收各种协议基类指针的容器，管理各种在构造函数当中添加的协议
 *          根据协议ID，可以将can报文数据解析为相关消息
 */
template <typename SensorType>
class MessageManager {
 public:
  /*
   * @brief constructor function
   */
  MessageManager() {}
  /*
   * @brief destructor function
   */
  virtual ~MessageManager() = default;

  /**
   * @brief parse data and store parsed info in protocol data
   * @param message_id the id of the message
   * @param data a pointer to the data array to be parsed
   * @param length the length of data array
   */
  virtual void Parse(const uint32_t message_id, const uint8_t *data,
                     int32_t length);

  void ClearSensorData();

  std::condition_variable *GetMutableCVar();

  /**
   * @brief get mutable protocol data by message id
   * @param message_id the id of the message
   * @return a pointer to the protocol data
   */
  ProtocolData<SensorType> *GetMutableProtocolDataById(
      const uint32_t message_id);

  /**
   * @brief get chassis detail. used lock_guard in this function to avoid
   * concurrent read/write issue.返回从can解析而来的消息
   * @param chassis_detail chassis_detail to be filled.
   */
  common::ErrorCode GetSensorData(SensorType *const sensor_data);

  /*
   * @brief reset send messages
   */
  void ResetSendMessages();

 protected:
  template <class T, bool need_check>
  void AddRecvProtocolData();

  template <class T, bool need_check>
  void AddSendProtocolData();

  std::vector<std::unique_ptr<ProtocolData<SensorType>>> send_protocol_data_; ///< 保存需要发送给can协议类对象指针的容器
  std::vector<std::unique_ptr<ProtocolData<SensorType>>> recv_protocol_data_; ///< 保存需要从can接收协议的容器

  std::unordered_map<uint32_t, ProtocolData<SensorType> *> protocol_data_map_; ///< 保存协议类ID号和指向该协议类堆对象指针的关联容器
  std::unordered_map<uint32_t, CheckIdArg> check_ids_; ///< 保存协议类ID号和该协议检查类的关联容器
  std::set<uint32_t> received_ids_; ///< 解析过的can协议

  std::mutex sensor_data_mutex_;
  SensorType sensor_data_; ///< 由接收的can报文解析而来的消息，反映了当前时刻从底层获得的底盘数据
  bool is_received_on_time_ = false;

  std::condition_variable cvar_;
};

/**
 * @brief 添加接收协议
 * @details 在接收协议容器当中创建一个指定的协议类的堆对象，并在关联容器当中加入该协议ID和堆对象的键值对
 * 
 * @tparam SensorType 
 * @tparam T 不同的协议类
 * @tparam need_check 是否需要检验
 */
template <typename SensorType>
template <class T, bool need_check>
void MessageManager<SensorType>::AddRecvProtocolData() {
  recv_protocol_data_.emplace_back(new T());
  auto *dt = recv_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

/**
 * @brief 添加发送协议
 * @details 在发送协议容器当中创建一个指定的协议类的堆对象，并在关联容器当中加入该协议ID和堆对象的键值对
 * 
 * @tparam SensorType 
 * @tparam T 不同的协议类
 * @tparam need_check 是否需要检验
 */
template <typename SensorType>
template <class T, bool need_check>
void MessageManager<SensorType>::AddSendProtocolData() {
  send_protocol_data_.emplace_back(new T());
  auto *dt = send_protocol_data_.back().get();
  if (dt == nullptr) {
    return;
  }
  protocol_data_map_[T::ID] = dt;
  if (need_check) {
    check_ids_[T::ID].period = dt->GetPeriod();
    check_ids_[T::ID].real_period = 0;
    check_ids_[T::ID].last_time = 0;
    check_ids_[T::ID].error_count = 0;
  }
}

/**
 * @brief 通过协议类ID找的对应的协议类堆对象的指针
 * 
 * @tparam SensorType 
 * @param message_id 
 * @return ProtocolData<SensorType>* 
 */
template <typename SensorType>
ProtocolData<SensorType>
    *MessageManager<SensorType>::GetMutableProtocolDataById(
        const uint32_t message_id) {
  if (protocol_data_map_.find(message_id) == protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << Byte::byte_to_hex(message_id);
    return nullptr;
  }
  return protocol_data_map_[message_id];
}

template <typename SensorType>
void MessageManager<SensorType>::Parse(const uint32_t message_id,
                                       const uint8_t *data, int32_t length) {
  // 根据id获得该can报文的协议类型堆对象指针
  ProtocolData<SensorType> *protocol_data =
      GetMutableProtocolDataById(message_id);
  if (protocol_data == nullptr) {
    return;
  }
  {
    // 通过协议类对象，将can报文数据解析为消息类型
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    protocol_data->Parse(data, length, &sensor_data_);
  }
  received_ids_.insert(message_id);
  // check if need to check period
  const auto it = check_ids_.find(message_id);
  if (it != check_ids_.end()) {
    const int64_t time = Time::Now().ToNanosecond() / 1e3;
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (static_cast<double>(it->second.real_period) >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

template <typename SensorType>
void MessageManager<SensorType>::ClearSensorData() {
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_data_.Clear();
}

template <typename SensorType>
std::condition_variable *MessageManager<SensorType>::GetMutableCVar() {
  return &cvar_;
}

template <typename SensorType>
ErrorCode MessageManager<SensorType>::GetSensorData(
    SensorType *const sensor_data) {
  if (sensor_data == nullptr) {
    AERROR << "Failed to get sensor_data due to nullptr.";
    return ErrorCode::CANBUS_ERROR;
  }
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);
  sensor_data->CopyFrom(sensor_data_);
  return ErrorCode::OK;
}

/**
 * @brief 重置协议类对象容器
 * @details 将协议类容器当中所有的智能指针重置
 * 
 * @tparam SensorType 
 */
template <typename SensorType>
void MessageManager<SensorType>::ResetSendMessages() {
  for (auto &protocol_data : send_protocol_data_) {
    if (protocol_data == nullptr) {
      AERROR << "Invalid protocol data.";
    } else {
      protocol_data->Reset();
    }
  }
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
