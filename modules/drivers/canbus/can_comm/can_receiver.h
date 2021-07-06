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
 * @brief Defines CanReceiver class.
 */

#pragma once

#include <atomic>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "cyber/common/macros.h"
#include "cyber/cyber.h"

#include "modules/common/proto/error_code.pb.h"

#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class CanReceiver
 * @brief CAN receiver.
 * @details 开启一个线程，不断接收底盘的can报文，根据其协议ID将其解析为消息类型并保存至数据成员中
 *          - MessageManager，根据协议ID将can报文解析为消息类型
 *          - CanClient，从can接收can报文
 */
template <typename SensorType>
class CanReceiver {
 public:
  /**
   * @brief Constructor.
   */
  CanReceiver() = default;

  /**
   * @brief Destructor.
   */
  virtual ~CanReceiver() = default;

  /**
   * @brief Initialize by a CAN client, message manager.
   * @details 将数据成员当中的CanClient和MessageManager指针指向参数列表当中指定的对象，并做空指针检查
   * @param can_client The CAN client to use for receiving messages.
   * @param pt_manager The message manager which can parse and
   *        get protocol data by message id.
   * @param enable_log If log the essential information during running.
   * @return An error code indicating the status of this initialization.
   */
  common::ErrorCode Init(CanClient *can_client,
                         MessageManager<SensorType> *pt_manager,
                         bool enable_log);

  /**
   * @brief Get the working status of this CAN receiver.
   *        To check if it is running.
   * @return If this CAN receiver is running.
   */
  bool IsRunning() const;

  /**
   * @brief Start the CAN receiver.
   * @return The error code indicating the status of this action.
   */
  ::apollo::common::ErrorCode Start();

  /**
   * @brief Stop the CAN receiver.
   */
  void Stop();

 private:
  void RecvThreadFunc();

  int32_t Start(bool is_blocked);

 private:
  std::atomic<bool> is_running_ = {false}; ///< 控制线程函数运行的标志
  // CanClient, MessageManager pointer life is managed by outer program
  CanClient *can_client_ = nullptr;
  MessageManager<SensorType> *pt_manager_ = nullptr;
  bool enable_log_ = false;
  bool is_init_ = false;
  std::future<void> async_result_;

  DISALLOW_COPY_AND_ASSIGN(CanReceiver);
};

template <typename SensorType>
::apollo::common::ErrorCode CanReceiver<SensorType>::Init(
    CanClient *can_client, MessageManager<SensorType> *pt_manager,
    bool enable_log) {
  can_client_ = can_client;
  pt_manager_ = pt_manager;
  enable_log_ = enable_log;
  if (can_client_ == nullptr) {
    AERROR << "Invalid can client.";
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }
  if (pt_manager_ == nullptr) {
    AERROR << "Invalid protocol manager.";
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }
  is_init_ = true;
  return ::apollo::common::ErrorCode::OK;
}

/**
 * @brief 线程的入口函数
 * @details 通过CanClient从can接收指定数目的can报文，并通过MessageManager，
 *          根据该报文的协议ID将所有接收到的can报文解析为ChassisDetail消息类型，
 *          存储在MessageManager的sensor_data_当中
 * @note 对共享变量sensor_data_进行了写操作
 * 
 * @tparam SensorType 
 */
template <typename SensorType>
void CanReceiver<SensorType>::RecvThreadFunc() {
  AINFO << "Can client receiver thread starts.";
  CHECK_NOTNULL(can_client_);
  CHECK_NOTNULL(pt_manager_);

  int32_t receive_error_count = 0;
  int32_t receive_none_count = 0;
  const int32_t ERROR_COUNT_MAX = 10;
  auto default_period = 10 * 1000;

  // 通过CanClient从can接收指定数目的can报文，并通过MessageManager将can报文解析为消息类型，存储与其数据成员当中
  while (IsRunning()) {
    std::vector<CanFrame> buf;
    int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
    // 从can中接收指定数目的can报文
    if (can_client_->Receive(&buf, &frame_num) !=
        ::apollo::common::ErrorCode::OK) {
      LOG_IF_EVERY_N(ERROR, receive_error_count++ > ERROR_COUNT_MAX,
                     ERROR_COUNT_MAX)
          << "Received " << receive_error_count << " error messages.";
      cyber::USleep(default_period);
      continue;
    }
    receive_error_count = 0;

    if (buf.size() != static_cast<size_t>(frame_num)) {
      AERROR_EVERY(100) << "Receiver buf size [" << buf.size()
                        << "] does not match can_client returned length["
                        << frame_num << "].";
    }

    if (frame_num == 0) {
      LOG_IF_EVERY_N(ERROR, receive_none_count++ > ERROR_COUNT_MAX,
                     ERROR_COUNT_MAX)
          << "Received " << receive_none_count << " empty messages.";
      cyber::USleep(default_period);
      continue;
    }
    receive_none_count = 0;

    for (const auto &frame : buf) {
      uint8_t len = frame.len;
      uint32_t uid = frame.id;
      const uint8_t *data = frame.data;
      pt_manager_->Parse(uid, data, len); // 对sensor_data_进行写操作
      if (enable_log_) {
        ADEBUG << "recv_can_frame#" << frame.CanFrameString();
      }
    }
    cyber::Yield();
  }
  AINFO << "Can client receiver thread stopped.";
}

template <typename SensorType>
bool CanReceiver<SensorType>::IsRunning() const {
  // load返回原子类型保存的变量
  return is_running_.load();
}

/**
 * @brief 开启线程
 * 
 * @tparam SensorType 
 * @return ::apollo::common::ErrorCode 
 */
template <typename SensorType>
::apollo::common::ErrorCode CanReceiver<SensorType>::Start() {
  if (is_init_ == false) {
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }
  // exchange改变原子类型变量的值，并返回原先的值
  is_running_.exchange(true);

  async_result_ = cyber::Async(&CanReceiver<SensorType>::RecvThreadFunc, this);
  return ::apollo::common::ErrorCode::OK;
}

/**
 * @brief 停止线程，并阻塞等待其停止
 * 
 * @tparam SensorType 
 */
template <typename SensorType>
void CanReceiver<SensorType>::Stop() {
  if (IsRunning()) {
    AINFO << "Stopping can client receiver ...";
    is_running_.exchange(false);
    // 阻塞等待线程函数返回
    async_result_.wait();
  } else {
    AINFO << "Can client receiver is not running.";
  }
  AINFO << "Can client receiver stopped [ok].";
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
