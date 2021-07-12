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

#include "modules/canbus/canbus_component.h"

#include "cyber/time/time.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

using apollo::common::ErrorCode;
using apollo::control::ControlCommand;
using apollo::cyber::Time;
using apollo::drivers::canbus::CanClientFactory;
using apollo::guardian::GuardianCommand;

namespace apollo {
namespace canbus {

std::string CanbusComponent::Name() const { return FLAGS_canbus_module_name; }

CanbusComponent::CanbusComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS) {}

/**
 * @brief 初始化canbus
 * @details 通过工厂模式，根据配置文件当中的参数创建指定的CanClient，以及指定车型的MessageManager和VehicleController
 *          初始化CanReceiver、CanSender和VehicleController，并注册相应的发布者和订阅者
 *          通过以上对象的方法，打开can端口并开启三个线程，用于发送、接收和检测
 * 
 * @return true 
 * @return false 
 */
bool CanbusComponent::Init() {
  // 从dag文件的指定路径的配置文件读取can卡配置
  if (!GetProtoConfig(&canbus_conf_)) {
    AERROR << "Unable to load canbus conf file: " << ConfigFilePath();
    return false;
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_canbus_conf_file;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  // Init can client
  // 创建单例的CanClientFactory类成员
  auto can_factory = CanClientFactory::Instance();
  // 注册接口名和创建其的函数
  can_factory->RegisterCanClients();
  // 基类指针实现继承多态
  // 根据CANCardParameter当中的信息创建一个can_client_堆对象，而不是在此文件当中指定，实现了松耦合
  can_client_ = can_factory->CreateCANClient(canbus_conf_.can_card_parameter());
  if (!can_client_) {
    AERROR << "Failed to create can client.";
    return false;
  }
  AINFO << "Can client is successfully created.";

  // 通过函数参数创建不同的车型抽象工厂堆对象
  VehicleFactory vehicle_factory;
  vehicle_factory.RegisterVehicleFactory();
  auto vehicle_object =
      vehicle_factory.CreateVehicle(canbus_conf_.vehicle_parameter());
  if (!vehicle_object) {
    AERROR << "Failed to create vehicle:";
    return false;
  }

  // 创建该车型的协议管理堆对象，创建的同时会创建该车型需要的协议类对象
  message_manager_ = vehicle_object->CreateMessageManager();
  if (message_manager_ == nullptr) {
    AERROR << "Failed to create message manager.";
    return false;
  }
  AINFO << "Message manager is successfully created.";

  // 通过配置文件当中指定创建的CanClient和MessageManager初始化CanReceiver
  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         canbus_conf_.enable_receiver_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can receiver.";
    return false;
  }
  AINFO << "The can receiver is successfully initialized.";

  // 通过配置文件当中指定创建的CanClient和MessageManager初始化CanSender
  if (can_sender_.Init(can_client_.get(), canbus_conf_.enable_sender_log()) !=
      ErrorCode::OK) {
    AERROR << "Failed to init can sender.";
    return false;
  }
  AINFO << "The can sender is successfully initialized.";

  // 创建指定车型的VehicleController堆对象，此时会在CanSender的容器当中添加所有需要发送的can报文
  vehicle_controller_ = vehicle_object->CreateVehicleController();
  if (vehicle_controller_ == nullptr) {
    AERROR << "Failed to create vehicle controller.";
    return false;
  }
  AINFO << "The vehicle controller is successfully created.";

  // VehicleController获得相关发送协议，并且在CanSender中创建相关协议的发送can报文
  if (vehicle_controller_->Init(canbus_conf_.vehicle_parameter(), &can_sender_,
                                message_manager_.get()) != ErrorCode::OK) {
    AERROR << "Failed to init vehicle controller.";
    return false;
  }

  AINFO << "The vehicle controller is successfully"
        << " initialized with canbus conf as : "
        << canbus_conf_.vehicle_parameter().ShortDebugString();

  // 设置订阅者的相关配置参数
  cyber::ReaderConfig guardian_cmd_reader_config;
  guardian_cmd_reader_config.channel_name = FLAGS_guardian_topic; //默认 /apollo/guardian
  guardian_cmd_reader_config.pending_queue_size =
      FLAGS_guardian_cmd_pending_queue_size; //默认 10

  cyber::ReaderConfig control_cmd_reader_config;
  control_cmd_reader_config.channel_name = FLAGS_control_command_topic; //默认 /apollo/control
  control_cmd_reader_config.pending_queue_size =
      FLAGS_control_cmd_pending_queue_size; //默认 10

  // 根据配置文件当中的命令行参数选择是否开启守护进程，创建订阅者，订阅控制信息并注册回调函数
  if (FLAGS_receive_guardian) {
    guardian_cmd_reader_ = node_->CreateReader<GuardianCommand>(
        guardian_cmd_reader_config,
        [this](const std::shared_ptr<GuardianCommand> &cmd) {
          ADEBUG << "Received guardian data: run canbus callback.";
          OnGuardianCommand(*cmd);
        });
  } else {
    control_command_reader_ = node_->CreateReader<ControlCommand>(
        control_cmd_reader_config,
        [this](const std::shared_ptr<ControlCommand> &cmd) {
          ADEBUG << "Received control data: run canbus callback.";
          OnControlCommand(*cmd);
        });
  }

  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic); // 默认 /apollo/canbus/chassis

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>(FLAGS_chassis_detail_topic); // 默认 /apollo/canbus/chassis_detail

  // 1. init and start the can card hardware
  // 打开对应的can端口
  if (can_client_->Start() != ErrorCode::OK) {
    AERROR << "Failed to start can client";
    return false;
  }
  AINFO << "Can client is started.";

  // 2. start receive first then send
  // CanReceiver开启一个线程，不断接收一定数目的can报文，
  // 并根据该报文的协议将其解析为消息类型，存储在MessageManager的数据成员sensor_data_当中
  if (can_receiver_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can receiver.";
    return false;
  }
  AINFO << "Can receiver is started.";

  // 3. start send
  // 开启一个线程，以各自规定的间隔，不断循环发送容器当中保存的所有的can报文
  if (can_sender_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can sender.";
    return false;
  }

  // 4. start controller
  // 开启一个线程，当can_sender_开启，执行发送can帧的线程时，
  // 以一定的频率，根据自动驾驶模式不断检查底盘模块是否正常，若连续出错的次数超出了阈值，则进入紧急模式
  if (!vehicle_controller_->Start()) {
    AERROR << "Failed to start vehicle controller.";
    return false;
  }

  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}

void CanbusComponent::Clear() {
  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();
  AINFO << "Cleanup Canbus component";
}

/**
 * @brief 获取底盘消息，并将其发送值话题/apollo/canbus/chassis
 * 
 */
void CanbusComponent::PublishChassis() {
  Chassis chassis = vehicle_controller_->chassis();
  // 添加首部信息
  common::util::FillHeader(node_->Name(), &chassis);
  chassis_writer_->Write(chassis);
  ADEBUG << chassis.ShortDebugString();
}

/**
 * @brief 获取详细底盘消息，并将其发送至话题/apollo/canbus/chassis_detail
 * 
 */
void CanbusComponent::PublishChassisDetail() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();
  chassis_detail_writer_->Write(chassis_detail);
}

/**
 * @brief 将从can获得的底盘信息发送至相关话题
 * @details 该函数会以一定的频率调用，并根据命令行参数可以选择是否发布详细底盘信息
 * 
 * @return true 发布成功
 * @return false 发布失败
 */
bool CanbusComponent::Proc() {
  PublishChassis();
  if (FLAGS_enable_chassis_detail_pub) {
    PublishChassisDetail();
  }
  return true;
}

/**
 * @brief 订阅控制话题的回调函数
 * @details 以一定频率接收话题当中的控制信息，并根据该控制信息，更新各个发送给can协议的报文
 *          在CanSender的线程当中，can报文会进行发送。
 *          该回调函数调用完成后，所有向车辆底层发送的can报文的数据会根据ControlCommand完成更新
 * 
 * @param control_command 订阅而来的控制消息
 */
void CanbusComponent::OnControlCommand(const ControlCommand &control_command) {
  int64_t current_timestamp = Time::Now().ToMicrosecond();
  // if command coming too soon, just ignore it.
  // 控制以FLAGS_min_cmd_interval * 1000的时间间隔接收话题当中的消息
  if (current_timestamp - last_timestamp_ < FLAGS_min_cmd_interval * 1000) {
    ADEBUG << "Control command comes too soon. Ignore.\n Required "
              "FLAGS_min_cmd_interval["
           << FLAGS_min_cmd_interval << "], actual time interval["
           << current_timestamp - last_timestamp_ << "].";
    return;
  }

  // 同意接收消息，更新时间
  last_timestamp_ = current_timestamp;
  ADEBUG << "Control_sequence_number:"
         << control_command.header().sequence_num() << ", Time_of_delay:"
         << current_timestamp -
                static_cast<int64_t>(control_command.header().timestamp_sec() *
                                     1e6)
         << " micro seconds";

  // vehicle_controller_更新,根据控制参数当中的物理量以及当前车辆驾驶模式,更新相应发送协议的物理量
  if (vehicle_controller_->Update(control_command) != ErrorCode::OK) {
    AERROR << "Failed to process callback function OnControlCommand because "
              "vehicle_controller_->Update error.";
    return;
  }
  // can_sender_更新，根据各个发送给can的协议类对象当中的物理量更新各自的can报文
  can_sender_.Update();
}

void CanbusComponent::OnGuardianCommand(
    const GuardianCommand &guardian_command) {
  OnControlCommand(guardian_command.control_command());
}

common::Status CanbusComponent::OnError(const std::string &error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  return ::apollo::common::Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace canbus
}  // namespace apollo
