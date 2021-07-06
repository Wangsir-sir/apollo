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
 * @brief Defines SenderMessage class and CanSender class.
 */

#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "cyber/common/macros.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace apollo {
namespace drivers {
namespace canbus {

/**
 * @class SenderMessage
 * @details 要发送的can报文数据，包括协议类型，can报文，发送间隔时间，can帧的ID号等
 * @brief This class defines the message to send.
 */
template <typename SensorType>
class SenderMessage {
 public:
  /**
   * @brief Constructor which takes message ID and protocol data.
   * @details 通过形参初始化can_frame_to_update_当中除了data的数据成员
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   */
  SenderMessage(const uint32_t message_id,
                ProtocolData<SensorType> *protocol_data);

  /**
   * @brief Constructor which takes message ID and protocol data and
   *        and indicator whether to initialize all bits of the .
   * @details 初始化can报文，根据协议默认的物理量初始can报文的数据内容
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   * @param init_with_one If it is true, then initialize all bits in
   *        the protocol data as one.
   */
  SenderMessage(const uint32_t message_id,
                ProtocolData<SensorType> *protocol_data, bool init_with_one);

  /**
   * @brief Destructor.
   */
  virtual ~SenderMessage() = default;

  /**
   * @brief Update the current period for sending messages by a difference.
   * @details 根据一次循环，当前剩余的发送时间间隔根据循环的时间间隔相应的减小
   *          若减小为0时，则恢复为默认的发送时间间隔
   * @param delta_period Update the current period by reducing delta_period.
   */
  void UpdateCurrPeriod(const int32_t delta_period);

  /**
   * @brief Update the protocol data. But the updating process depends on
   *        the real type of protocol data which inherites ProtocolData.
   * @note 对can_frame_to_send_的写操作
   * @warning 每当can协议对象的数据成员发生变换时都要调用该函数，更新can报文
   */
  void Update();

  /**
   * @brief Get the CAN frame to send.
   *        返回要发送的can帧
   * @note 该函数是线程安全函数, 对can_frame_to_send_的读操作
   * @return The CAN frame to send.
   */
  struct CanFrame CanFrame();

  /**
   * @brief Get the message ID.
   * @return The message ID.
   */
  uint32_t message_id() const;

  /**
   * @brief Get the current period to send messages. It may be different from
   *        the period from protocol data by updating.
   * @return The current period.
   */
  int32_t curr_period() const;

 private:
  uint32_t message_id_ = 0; ///can报文的协议ID
  ProtocolData<SensorType> *protocol_data_ = nullptr; ///< 发送can数据的协议类型

  int32_t period_ = 0; ///< 该can帧协议规定的发送间隔
  int32_t curr_period_ = 0; ///< 当前can帧需要发送的剩余时间间隔

 private:
  static std::mutex mutex_; ///< 保护can_frame_to_send_的互斥量
  // 在主线程中发生写操作，在其他线程中发生读操作
  struct CanFrame can_frame_to_send_; ///< 要发送的can帧（需要保护的共享变量）
  struct CanFrame can_frame_to_update_; ///< 被协议更新的can帧
};

/**
 * @class CanSender
 * @brief CAN sender.
 * @details 在容器当中保存各种发送协议的can报文，并开启一个线程，
 *          不断循环将其以对应的协议的间隔发送至can网络
 *          - CanClient，向can发送can报文
 *          - std::vector<SenderMessage<SensorType>>，保存发送can报文的容器，
 *            在各个车型的VehicleController派生类的Init函数当中，根据各自的需要进行初始化
 */
template <typename SensorType>
class CanSender {
 public:
  /**
   * @brief Constructor.
   */
  CanSender() = default;

  /**
   * @brief Destructor.
   */
  virtual ~CanSender() = default;

  /**
   * @brief Initialize by a CAN client based on its brand.
   * @param can_client The CAN client to use for sending messages.
   * @param enable_log whether enable record the send can frame log
   * @return An error code indicating the status of this initialization.
   */
  common::ErrorCode Init(CanClient *can_client, bool enable_log);

  /**
   * @brief Add a message with its ID, protocol data.
   * @details 在发送can帧容器当中创建要发送的can报文SenderMessage
   * @param message_id The message ID.
   * @param protocol_data A pointer of ProtocolData
   *        which contains the content to send.
   * @param init_with_one If it is true, then initialize all bits in
   *        the protocol data as one. By default, it is false.
   */
  void AddMessage(uint32_t message_id, ProtocolData<SensorType> *protocol_data,
                  bool init_with_one = false);

  /**
   * @brief Start the CAN sender.
   * @details 开启一个线程，发送要发送的can帧
   * @return The error code indicating the status of this action.
   */
  apollo::common::ErrorCode Start();

  /**
   * @brief Update the protocol data based the types.根据各自的协议更新各个要发送的can帧
   * @note 对can_frame_to_send_的写操作
   * @warning 每当发送can协议对象的物理量发生变换时都要调用该函数，更新所有发送can报文
   */
  void Update();

  /**
   * @brief Stop the CAN sender.
   * @details 通过is_running_使得线程的入口函数返回，并回收线程资源
   * @warning 需要调用stop结束线程并回收线程资源
   */
  void Stop();

  /**
   * @brief Get the working status of this CAN sender.
   *        To check if it is running.
   * @return If this CAN sender is running.
   */
  bool IsRunning() const;
  bool enable_log() const;

  FRIEND_TEST(CanSenderTest, OneRunCase);

 private:
  void PowerSendThreadFunc(); ///< 线程的入口函数

  bool NeedSend(const SenderMessage<SensorType> &msg,
                const int32_t delta_period);
  bool is_init_ = false; ///< 保证只初始化一次
  bool is_running_ = false; ///< 发送can帧的线程是否在执行。控制线程的while循环，false时线程的入口函数会返回

  CanClient *can_client_ = nullptr;  // Owned by global canbus.cc 通过CanClient提供的方法发送can帧
  std::vector<SenderMessage<SensorType>> send_messages_; ///< 保存要发送的can报文的容器
  std::unique_ptr<std::thread> thread_;
  bool enable_log_ = false;

  DISALLOW_COPY_AND_ASSIGN(CanSender);
};

const uint32_t kSenderInterval = 6000;

/********************************* SenderMessage定义 *************************************/

template <typename SensorType>
std::mutex SenderMessage<SensorType>::mutex_;

template <typename SensorType>
SenderMessage<SensorType>::SenderMessage(
    const uint32_t message_id, ProtocolData<SensorType> *protocol_data)
    : SenderMessage(message_id, protocol_data, false) {}

template <typename SensorType>
SenderMessage<SensorType>::SenderMessage(
    const uint32_t message_id, ProtocolData<SensorType> *protocol_data,
    bool init_with_one)
    : message_id_(message_id), protocol_data_(protocol_data) {
  if (init_with_one) {
    for (int32_t i = 0; i < protocol_data->GetLength(); ++i) {
      can_frame_to_update_.data[i] = 0xFF;
    }
  }
  int32_t len = protocol_data_->GetLength();

  can_frame_to_update_.id = message_id_;
  can_frame_to_update_.len = static_cast<uint8_t>(len);

  period_ = protocol_data_->GetPeriod();
  curr_period_ = period_;

  Update();// 主线程当中can_frame_to_send_的初始化
}

template <typename SensorType>
void SenderMessage<SensorType>::UpdateCurrPeriod(const int32_t period_delta) {
  // curr_period_也在线程函数当中进行了写操作，为何不需要互斥量保护？
  curr_period_ -= period_delta;
  if (curr_period_ <= 0) {
    curr_period_ = period_;
  }
}

template <typename SensorType>
void SenderMessage<SensorType>::Update() {
  if (protocol_data_ == nullptr) {
    AERROR << "Attention: ProtocolData is nullptr!";
    return;
  }
  // 不同的协议根据其保存的物理量更新can报文
  protocol_data_->UpdateData(can_frame_to_update_.data);

  // 难道不需要线程同步吗？
  // 会不会存在若can_frame_to_send_还没有被更新时线程函数就返回了can_frame_to_send_的情况
  // 初步认定不需要，循环间隔时间应该足以满足can_frame_to_send_初始化
  std::lock_guard<std::mutex> lock(mutex_);
  can_frame_to_send_ = can_frame_to_update_; // 对can_frame_to_send_的写操作
}

template <typename SensorType>
uint32_t SenderMessage<SensorType>::message_id() const {
  return message_id_;
}

template <typename SensorType>
struct CanFrame SenderMessage<SensorType>::CanFrame() {
  std::lock_guard<std::mutex> lock(mutex_);
  return can_frame_to_send_;// 对can_frame_to_send_的读操作
}

template <typename SensorType>
int32_t SenderMessage<SensorType>::curr_period() const {
  return curr_period_;
}

/********************************* CanSender定义 *************************************/

/**
 * @brief 线程的入口函数
 * @details 不断循环发送容器当中保存的所有的can报文，并以各自规定的间隔发送
 * @note 在该线程中会调用每个SenderMessage的CanFrame方法，会读其数据成员can_frame_to_send_
 *       在主线程当中的AddMessage当中，会调用SenderMessage的构造函数，也会初始化其数据成员can_frame_to_send_
 *       为避免数据竞争，SenderMessage中的对can_frame_to_send_进行访问的方法都设计为线程安全函数，调用是不需要互斥量保护
 * 
 * @tparam SensorType 
 */
template <typename SensorType>
void CanSender<SensorType>::PowerSendThreadFunc() {
  // 设置线程调度优先级？
  CHECK_NOTNULL(can_client_);
  sched_param sch;
  sch.sched_priority = 99;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

  const int32_t INIT_PERIOD = 5000;  // 5ms 初始循环间隔时间
  int32_t delta_period = INIT_PERIOD; // 每次for循环的时间间隔
  int32_t new_delta_period = INIT_PERIOD; // 需要更新的for循环时间间隔

  int64_t tm_start = 0;
  int64_t tm_end = 0;
  int64_t sleep_interval = 0;

  AINFO << "Can client sender thread starts.";

  // 以每个can帧协议规定的间隔，不断循环发送该can帧
  while (is_running_) {
    tm_start = cyber::Time::Now().ToNanosecond() / 1e3;
    new_delta_period = INIT_PERIOD;

    for (auto &message : send_messages_) {
      /**
       * @note 控制该can帧以规定的时间间隔发送
       *       delta_period会控制循环间隔，每次循环该can帧的剩余发送间隔会相应的减少
       *       当剩余间隔小于循环间隔时，会进行发送，并更新循环间隔为较小的值，提高精确程度
       */
      bool need_send = NeedSend(message, delta_period);
      message.UpdateCurrPeriod(delta_period); // 对curr_period_的写操作
      new_delta_period = std::min(new_delta_period, message.curr_period());

      if (!need_send) {
        continue;
      }
      // 将该can帧发送给can
      std::vector<CanFrame> can_frames;
      CanFrame can_frame = message.CanFrame(); // 对can_frame_to_send_的读操作
      can_frames.push_back(can_frame);
      if (can_client_->SendSingleFrame(can_frames) != common::ErrorCode::OK) {
        AERROR << "Send msg failed:" << can_frame.CanFrameString();
      }
      if (enable_log()) {
        ADEBUG << "send_can_frame#" << can_frame.CanFrameString();
      }
    }
    delta_period = new_delta_period;
    tm_end = cyber::Time::Now().ToNanosecond() / 1e3;
    sleep_interval = delta_period - (tm_end - tm_start);

    // 停顿，使得每次循环的时间为delta_period
    if (sleep_interval > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_interval));
    } else {
      // 循环经过的时间超过了delta_period
      // do not sleep
      AWARN << "Too much time for calculation: " << tm_end - tm_start
            << "us is more than minimum period: " << delta_period << "us";
    }
  }
  AINFO << "Can client sender thread stopped!";
}

template <typename SensorType>
common::ErrorCode CanSender<SensorType>::Init(CanClient *can_client,
                                              bool enable_log) {
  if (is_init_) {
    AERROR << "Duplicated Init request.";
    return common::ErrorCode::CANBUS_ERROR;
  }
  if (can_client == nullptr) {
    AERROR << "Invalid can client.";
    return common::ErrorCode::CANBUS_ERROR;
  }
  is_init_ = true;
  can_client_ = can_client;
  enable_log_ = enable_log;
  return common::ErrorCode::OK;
}

template <typename SensorType>
void CanSender<SensorType>::AddMessage(uint32_t message_id,
                                       ProtocolData<SensorType> *protocol_data,
                                       bool init_with_one) {
  if (protocol_data == nullptr) {
    AERROR << "invalid protocol data.";
    return;
  }
  send_messages_.emplace_back(
      SenderMessage<SensorType>(message_id, protocol_data, init_with_one));
  AINFO << "Add send message:" << std::hex << message_id;
}

template <typename SensorType>
common::ErrorCode CanSender<SensorType>::Start() {
  if (is_running_) {
    AERROR << "Cansender has already started.";
    return common::ErrorCode::CANBUS_ERROR;
  }
  is_running_ = true;
  thread_.reset(new std::thread([this] { PowerSendThreadFunc(); }));

  return common::ErrorCode::OK;
}

template <typename SensorType>
void CanSender<SensorType>::Update() {
  for (auto &message : send_messages_) {
    message.Update();
  }
}

template <typename SensorType>
void CanSender<SensorType>::Stop() {
  if (is_running_) {
    AINFO << "Stopping can sender ...";
    is_running_ = false;
    if (thread_ != nullptr && thread_->joinable()) {
      thread_->join();
    }
    thread_.reset();
  } else {
    AERROR << "CanSender is not running.";
  }

  AINFO << "Can client sender stopped [ok].";
}

template <typename SensorType>
bool CanSender<SensorType>::IsRunning() const {
  return is_running_;
}

template <typename SensorType>
bool CanSender<SensorType>::enable_log() const {
  return enable_log_;
}

/**
 * @brief 判断该can帧是否需要发送
 * @details 若剩余发送间隔小于等于本次循环的间隔时，允许发送，否则不允许发送，保证该帧以一定时间间隔发送
 * 
 * @tparam SensorType 
 * @param msg 要发送的can帧
 * @param delta_period 时间间隔
 * @return true 需要发送
 * @return false 不需要发送
 */
template <typename SensorType>
bool CanSender<SensorType>::NeedSend(const SenderMessage<SensorType> &msg,
                                     const int32_t delta_period) {
  return msg.curr_period() <= delta_period;
}

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
