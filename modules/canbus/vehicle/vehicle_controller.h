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
 * @file vehicle_controller.h
 * @brief The class of VehicleController
 */

#pragma once

#include <unordered_map>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

using ::apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;

/**
 * @class VehicleController
 *
 * @brief This is the interface class of vehicle controller. It defines pure
 * virtual functions, and also some implemented common functions.
 * @details 所有控制器的基类，该类包含向can发送报文的协议类堆对象指针，可以通过其定义的一系列方法修改发送协议的物理量
 *          不同的车型继承该类，并在派生类当中定义相关车型的的发送协议指针，
 *          通过Update方法，根据控制消息参数修改CanSender对应发送协议的物理量
 *          - 通过MessageManager对象根据不同的协议ID找到该车型相关的发送协议堆对象，初始化指针
 *            并通过其GetSensorData方法，获得从can接收解析而来的底盘消息，获得当前底盘的信息
 *          - 通过CanSender，根据相应的协议在当中创建相应的can发送报文，并在修改相关can协议物理量时通过CanSender更新其保存的所有can报文
 */
class VehicleController {
 public:
  virtual ~VehicleController() = default;

  /**
   * @brief initialize the vehicle controller.
   * @param can_sender a pointer to canbus sender.
   * @param message_manager a pointer to the message_manager.
   * @return error_code
   */
  virtual common::ErrorCode Init(
      const VehicleParameter &params,
      CanSender<ChassisDetail> *const can_sender,
      MessageManager<ChassisDetail> *const message_manager) = 0;

  /**
   * @brief start the vehicle controller.
   * @return true if successfully started.
   */
  virtual bool Start() = 0;

  /**
   * @brief stop the vehicle controller.
   */
  virtual void Stop() = 0;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  virtual Chassis chassis() = 0;

  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
  virtual common::ErrorCode Update(const control::ControlCommand &command);

  /**
   * @brief set vehicle to appointed driving mode.
   * @param driving mode to be appointed.
   * @return error_code
   */
  virtual common::ErrorCode SetDrivingMode(
      const Chassis::DrivingMode &driving_mode);

 private:
  /*
   * @brief main logical function for operation the car enter or exit the auto
   * driving
   */
  virtual void Emergency() = 0;

  virtual common::ErrorCode EnableAutoMode() = 0;
  virtual common::ErrorCode DisableAutoMode() = 0;
  virtual common::ErrorCode EnableSteeringOnlyMode() = 0;
  virtual common::ErrorCode EnableSpeedOnlyMode() = 0;

  /**
   * @brief NEUTRAL, REVERSE, DRIVE
   *        通过修改相关协议类的数据成员，设置车辆的档位
   */
  virtual void Gear(Chassis::GearPosition state) = 0;

  /**
   * @brief detail function for auto driving brake with new acceleration
   * acceleration:0.00~99.99, unit:%
   *        通过修改相关协议类的数据成员，设置刹车开度
   */
  virtual void Brake(double acceleration) = 0;

  /**
   * @brief drive with old acceleration gas:0.00~99.99 unit:%
   *        通过修改相关协议类的数据成员，设置油门开度
   */
  virtual void Throttle(double throttle) = 0;

  /**
   * @brief drive with new acceleration/deceleration:-7.0~7.0, unit:m/s^2,
   * acc:-7.0~7.0, unit:m/s^2
   *        通过修改相关协议类的数据成员，设置加速度
   */
  virtual void Acceleration(double acc) = 0;

  /**
   * @brief steering with old angle speed angle:-99.99~0.00~99.99, unit:%,
   * left:+, right:-
   *        通过修改相关协议类的数据成员，设置前轮转角
   */
  virtual void Steer(double angle) = 0;

  /**
   * @brief steering with new angle speed angle:-99.99~0.00~99.99, unit:%,
   * left:+, right:- angle_spd:0.00~99.99, unit:deg/s
   *        通过修改相关协议类的数据成员，设置前轮转角和转向速度
   */
  virtual void Steer(double angle, double angle_spd) = 0;

  /**
   * @brief set Electrical Park Brake
   */
  virtual void SetEpbBreak(const control::ControlCommand &command) = 0;
  virtual void SetBeam(const control::ControlCommand &command) = 0;
  virtual void SetHorn(const control::ControlCommand &command) = 0;
  virtual void SetTurningSignal(const control::ControlCommand &command) = 0;

  virtual void SetLimits() {} // 该函数没有实现

 protected:
  virtual Chassis::DrivingMode driving_mode();
  virtual void set_driving_mode(const Chassis::DrivingMode &driving_mode);

 protected:
  canbus::VehicleParameter params_; ///< 车辆参数
  common::VehicleParam vehicle_params_; ///< 车辆配置参数
  CanSender<ChassisDetail> *can_sender_ = nullptr; ///< 发送can数据类，用于当相应的协议类的数据成员发生变化时更新can报文
  MessageManager<ChassisDetail> *message_manager_ = nullptr; ///< 管理该车型can协议
  bool is_initialized_ = false;  // own by derviative concrete controller
  Chassis::DrivingMode driving_mode_ = Chassis::COMPLETE_MANUAL; ///< 车辆当前的驾驶模式
  bool is_reset_ = false;  // reset command from control command
  std::mutex mode_mutex_;  // only use in this base class
};

}  // namespace canbus
}  // namespace apollo
