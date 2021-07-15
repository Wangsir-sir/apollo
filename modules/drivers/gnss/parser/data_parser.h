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

#pragma once

#include <memory>
#include <string>

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj_api.h>

#include "cyber/cyber.h"
#include "modules/transform/transform_broadcaster.h"

#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"

#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

/**
 * @brief 导航数据解析类
 * @details 包含一个解析器，将设备读取的GPS原始数据解析为对应的消息类型，
 *          然后发布至相应的话题
 * 
 */
class DataParser {
 public:
  using MessagePtr = ::google::protobuf::Message *;
  DataParser(const config::Config &config,
             const std::shared_ptr<apollo::cyber::Node> &node);
  ~DataParser() {}
  bool Init();
  void ParseRawData(const std::string &msg);

 private:
  void DispatchMessage(Parser::MessageType type, MessagePtr message);
  void PublishInsStat(const MessagePtr message);
  void PublishOdometry(const MessagePtr message);
  void PublishCorrimu(const MessagePtr message);
  void PublishImu(const MessagePtr message);
  void PublishBestpos(const MessagePtr message);
  void PublishEphemeris(const MessagePtr message);
  void PublishObservation(const MessagePtr message);
  void PublishHeading(const MessagePtr message);
  void CheckInsStatus(Ins *ins);
  void CheckGnssStatus(Gnss *gnss);
  void GpsToTransformStamped(
      const std::shared_ptr<apollo::localization::Gps> &gps,
      apollo::transform::TransformStamped *transform);

  bool init_flag_ = false; ///< 数据解析类初始化标志
  config::Config config_;
  std::unique_ptr<Parser> data_parser_; ///< 解析器
  apollo::transform::TransformBroadcaster tf_broadcaster_;

  GnssStatus gnss_status_; ///< 卫星导航状态
  InsStatus ins_status_; ///< 惯性导航状态
  uint32_t ins_status_record_ = static_cast<uint32_t>(0);
  projPJ wgs84pj_source_;
  projPJ utm_target_;

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<GnssStatus>> gnssstatus_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<InsStatus>> insstatus_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<GnssBestPose>> gnssbestpose_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::CorrectedImu>>
      corrimu_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<Imu>> rawimu_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::Gps>>
      gps_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<InsStat>> insstat_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<GnssEphemeris>> gnssephemeris_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<EpochObservation>>
      epochobservation_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<Heading>> heading_writer_ = nullptr;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
