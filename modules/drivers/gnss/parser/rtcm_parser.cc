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

#include "modules/drivers/gnss/parser/rtcm_parser.h"

#include <memory>

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm3_parser.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::drivers::gnss::EpochObservation;
using ::apollo::drivers::gnss::GnssEphemeris;

RtcmParser::RtcmParser(const config::Config& config,
                       const std::shared_ptr<apollo::cyber::Node>& node)
    : config_(config), node_(node) {}

/**
 * @brief 初始化
 * @details 创建解析器，注册相应话题
 * 
 * @return true 
 * @return false 
 */
bool RtcmParser::Init() {
  rtcm_parser_.reset(new Rtcm3Parser(true));

  if (!rtcm_parser_) {
    AERROR << "Failed to create rtcm parser.";
    return false;
  }

  // /apollo/sensor/gnss/rtk_eph 发布GNSS天体位置表信息
  gnssephemeris_writer_ =
      node_->CreateWriter<GnssEphemeris>(FLAGS_gnss_rtk_eph_topic);
  // /apollo/sensor/gnss/rtk_obs 发布GNSS观察信息
  epochobservation_writer_ =
      node_->CreateWriter<EpochObservation>(FLAGS_gnss_rtk_obs_topic);
  init_flag_ = true;
  return true;
}

/**
 * @brief 解析RTK数据
 * @details 根据RTK设备流中读取数据，并解析为相关的消息并发布至对应的话题
 * 
 * @param msg 
 */
void RtcmParser::ParseRtcmData(const std::string& msg) {
  if (!init_flag_) {
    return;
  }

  // 更新待解析数据
  rtcm_parser_->Update(msg);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  // 根据RTK设备流中读取数据，并解析为相关的消息
  while (cyber::OK()) {
    type = rtcm_parser_->GetMessage(&msg_ptr);
    if (type == Parser::MessageType::NONE) {
      break;
    }
    // 发布至对应的话题
    DispatchMessage(type, msg_ptr);
  }
}

void RtcmParser::DispatchMessage(Parser::MessageType type, MessagePtr message) {
  switch (type) {
    case Parser::MessageType::EPHEMERIDES:
      PublishEphemeris(message);
      break;

    case Parser::MessageType::OBSERVATION:
      PublishObservation(message);
      break;

    default:
      break;
  }
}

void RtcmParser::PublishEphemeris(const MessagePtr& message) {
  auto eph = std::make_shared<GnssEphemeris>(*As<GnssEphemeris>(message));
  gnssephemeris_writer_->Write(eph);
}

void RtcmParser::PublishObservation(const MessagePtr& message) {
  auto observation =
      std::make_shared<EpochObservation>(*As<EpochObservation>(message));
  epochobservation_writer_->Write(observation);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
