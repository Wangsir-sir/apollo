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

#include "modules/drivers/gnss/parser/data_parser.h"

#include <cmath>
#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "boost/array.hpp"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/localization/proto/imu.pb.h"

#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::localization::CorrectedImu;
using ::apollo::localization::Gps;

using apollo::transform::TransformStamped;

namespace {

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";

// covariance data for pose if can not get from novatel inscov topic
static const boost::array<double, 36> POSE_COVAR = {
    2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

/**
 * @brief 解析器的工厂类方法
 * @details 根据配置参数中数据设备流的format参数创建对应的解析器
 * 
 * @param config 配置参数
 * @param is_base_station 
 * @return Parser* 解析器
 */
Parser *CreateParser(config::Config config, bool is_base_station = false) {
  // TODO 解析器的工厂方法
  switch (config.data().format()) {
    case config::Stream::NOVATEL_BINARY:
      return Parser::CreateNovatel(config);

    default:
      return nullptr;
  }
}

}  // namespace

/**
 * @brief Construct a new Data Parser:: Data Parser object
 * @details description
 * 
 * @param config 
 * @param node 
 */
DataParser::DataParser(const config::Config &config,
                       const std::shared_ptr<apollo::cyber::Node> &node)
    : config_(config), tf_broadcaster_(node), node_(node) {
  std::string utm_target_param;

  wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
  utm_target_ = pj_init_plus(config_.proj4_text().c_str());
  // 初始化卫星导航状态
  gnss_status_.set_solution_status(0);
  gnss_status_.set_num_sats(0);
  gnss_status_.set_position_type(0);
  gnss_status_.set_solution_completed(false);
  // 初始化卫星导航状态
  ins_status_.set_type(InsStatus::INVALID);
}

/**
 * @brief 初始化
 * @details 初始化发布者话题，根据配置参数初始化解析器
 * 
 * @return true 
 * @return false 
 */
bool DataParser::Init() {
  ins_status_.mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  gnss_status_.mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());

  // /apollo/sensor/gnss/gnss_status 卫星导航状态
  gnssstatus_writer_ = node_->CreateWriter<GnssStatus>(FLAGS_gnss_status_topic);
  // /apollo/sensor/gnss/ins_status 惯性导航状态
  insstatus_writer_ = node_->CreateWriter<InsStatus>(FLAGS_ins_status_topic);
  // /apollo/sensor/gnss/best_pose 大地坐标系GNSS定位信息
  gnssbestpose_writer_ =
      node_->CreateWriter<GnssBestPose>(FLAGS_gnss_best_pose_topic);
  // /apollo/sensor/gnss/corrected_imu ENU坐标系下的IMU消息
  corrimu_writer_ = node_->CreateWriter<CorrectedImu>(FLAGS_imu_topic);
  // /apollo/sensor/gnss/ins_stat InsStat
  insstat_writer_ = node_->CreateWriter<InsStat>(FLAGS_ins_stat_topic);
  // /apollo/sensor/gnss/rtk_eph GNSS卫星位置表信息
  gnssephemeris_writer_ =
      node_->CreateWriter<GnssEphemeris>(FLAGS_gnss_rtk_eph_topic);
  // /apollo/sensor/gnss/rtk_obs 一轮的观测结果
  epochobservation_writer_ =
      node_->CreateWriter<EpochObservation>(FLAGS_gnss_rtk_obs_topic);
  // /apollo/sensor/gnss/heading Heading
  heading_writer_ = node_->CreateWriter<Heading>(FLAGS_heading_topic);
  // /apollo/sensor/gnss/imu ENU坐标系下的IMU消息
  rawimu_writer_ = node_->CreateWriter<Imu>(FLAGS_raw_imu_topic);
  // /apollo/sensor/gnss/odometry ENU坐标系坐标系下的定位消息
  gps_writer_ = node_->CreateWriter<Gps>(FLAGS_gps_topic);

  // 发布初始化后的卫星导航状态和惯性导航状态
  common::util::FillHeader("gnss", &ins_status_);
  insstatus_writer_->Write(ins_status_);
  common::util::FillHeader("gnss", &gnss_status_);
  gnssstatus_writer_->Write(gnss_status_);

  AINFO << "Creating data parser of format: " << config_.data().format();
  data_parser_.reset(CreateParser(config_, false));
  if (!data_parser_) {
    AFATAL << "Failed to create data parser.";
    return false;
  }

  init_flag_ = true;
  return true;
}

/**
 * @brief 解析GPS原始数据。
 * @details 不断解析组合导航数据,并发布至相应的话题
 * 
 * @param msg 
 */
void DataParser::ParseRawData(const std::string &msg) {
  if (!init_flag_) {
    AERROR << "Data parser not init.";
    return;
  }

  // 更新解析器要解析的数据
  data_parser_->Update(msg);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  // 不断解析组合导航数据,并发布至相应的话题
  while (cyber::OK()) {
    // 将数据指针的内容解析为组合导航消息
    type = data_parser_->GetMessage(&msg_ptr);
    if (type == Parser::MessageType::NONE) {
      break;
    }
    // 根据消息的类型,对消息进行对应的处理后,发布至相应的话题
    DispatchMessage(type, msg_ptr);
  }
}

/**
 * @brief 根据融合了GNSS的惯性导航结果得到惯性导航状态，并发布至相关话题
 * 
 * @param ins 
 */
void DataParser::CheckInsStatus(::apollo::drivers::gnss::Ins *ins) {
  static double last_notify = cyber::Time().Now().ToSecond();
  double now = cyber::Time().Now().ToSecond();
  if (ins_status_record_ != static_cast<uint32_t>(ins->type()) ||
      (now - last_notify) > 1.0) {
    last_notify = now;
    ins_status_record_ = static_cast<uint32_t>(ins->type());
    switch (ins->type()) {
      case apollo::drivers::gnss::Ins::GOOD:
        ins_status_.set_type(apollo::drivers::gnss::InsStatus::GOOD);
        break;

      case apollo::drivers::gnss::Ins::CONVERGING:
        ins_status_.set_type(apollo::drivers::gnss::InsStatus::CONVERGING);
        break;

      case apollo::drivers::gnss::Ins::INVALID:
      default:
        ins_status_.set_type(apollo::drivers::gnss::InsStatus::INVALID);
        break;
    }

    common::util::FillHeader("gnss", &ins_status_);
    insstatus_writer_->Write(ins_status_);
  }
}

/**
 * @brief 根据没有经过IMU数据融合的GNSS结果，得到卫星导航状态，并发布至相关话题
 * 
 * @param gnss 
 */
void DataParser::CheckGnssStatus(::apollo::drivers::gnss::Gnss *gnss) {
  gnss_status_.set_solution_status(
      static_cast<uint32_t>(gnss->solution_status()));
  gnss_status_.set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
  gnss_status_.set_position_type(static_cast<uint32_t>(gnss->position_type()));

  if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
    gnss_status_.set_solution_completed(true);
  } else {
    gnss_status_.set_solution_completed(false);
  }
  common::util::FillHeader("gnss", &gnss_status_);
  gnssstatus_writer_->Write(gnss_status_);
}

/**
 * @brief 根据消息的类型,对消息进行对应的处理后,发布至相应的话题
 * 
 * @param type 
 * @param message 
 */
void DataParser::DispatchMessage(Parser::MessageType type, MessagePtr message) {
  switch (type) {
    // 根据没有经过IMU数据融合的GNSS结果，得到卫星导航状态，并发布至相关话题
    case Parser::MessageType::GNSS:
      CheckGnssStatus(As<::apollo::drivers::gnss::Gnss>(message));
      break;

    // 将GNSS定位信息发布至相关话题
    case Parser::MessageType::BEST_GNSS_POS:
      PublishBestpos(message);
      break;

    // 将IMU消息发布至相关话题
    case Parser::MessageType::IMU:
      PublishImu(message);
      break;

    // 根据融合了GNSS的惯性导航结果得到融合了GNSS的惯性导航结果，并发布至相关话题
    // 根据IMU消息，进行一定处理后填充CorrectedImu，并发布至相关话题
    // 根据IMU消息进行一定的处理，填充GPS消息，并发布至相关话题
    case Parser::MessageType::INS:
      CheckInsStatus(As<::apollo::drivers::gnss::Ins>(message));
      PublishCorrimu(message);
      PublishOdometry(message);
      break;

    // 将InsStat发布至相关话题
    case Parser::MessageType::INS_STAT:
      PublishInsStat(message);
      break;

    // 将GNSS卫星位置表信息发布至相关话题
    case Parser::MessageType::BDSEPHEMERIDES:
    case Parser::MessageType::GPSEPHEMERIDES:
    case Parser::MessageType::GLOEPHEMERIDES:
      PublishEphemeris(message);
      break;

    // 将一轮的观测结果发布至相关话题
    case Parser::MessageType::OBSERVATION:
      PublishObservation(message);
      break;

    // 将Heading发布至相关话题
    case Parser::MessageType::HEADING:
      PublishHeading(message);
      break;

    default:
      break;
  }
}

/**
 * @brief 将InsStat发布至相关话题
 * 
 * @param message 
 */
void DataParser::PublishInsStat(const MessagePtr message) {
  auto ins_stat = std::make_shared<InsStat>(*As<InsStat>(message));
  common::util::FillHeader("gnss", ins_stat.get());
  insstat_writer_->Write(ins_stat);
}

/**
 * @brief 将GNSS定位信息发布至相关话题
 * 
 * @param message 
 */
void DataParser::PublishBestpos(const MessagePtr message) {
  auto bestpos = std::make_shared<GnssBestPose>(*As<GnssBestPose>(message));
  common::util::FillHeader("gnss", bestpos.get());
  gnssbestpose_writer_->Write(bestpos);
}

/**
 * @brief 对融合了GNSS的惯性导航结果的x轴的数据取反后发布至相关话题
 * @details 将线加速度和角速度FLU坐标系转换为ENU坐标系
 * 
 * @param message 
 */
void DataParser::PublishImu(const MessagePtr message) {
  auto raw_imu = std::make_shared<Imu>(*As<Imu>(message));
  Imu *imu = As<Imu>(message);

  raw_imu->mutable_linear_acceleration()->set_x(
      -imu->linear_acceleration().y());
  raw_imu->mutable_linear_acceleration()->set_y(imu->linear_acceleration().x());
  raw_imu->mutable_linear_acceleration()->set_z(imu->linear_acceleration().z());

  raw_imu->mutable_angular_velocity()->set_x(-imu->angular_velocity().y());
  raw_imu->mutable_angular_velocity()->set_y(imu->angular_velocity().x());
  raw_imu->mutable_angular_velocity()->set_z(imu->angular_velocity().z());

  common::util::FillHeader("gnss", raw_imu.get());
  rawimu_writer_->Write(raw_imu);
}

/**
 * @brief 对融合了GNSS的惯性导航结果进行一定的处理，填充GPS消息，并发布至相关话题
 * @details 将大地坐标系转换为UTM下的局部坐标系ENU
 * 
 * @param message 
 */
void DataParser::PublishOdometry(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  auto gps = std::make_shared<Gps>();

  double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
  gps->mutable_header()->set_timestamp_sec(unix_sec);
  auto *gps_msg = gps->mutable_localization();

  // 1. pose xyz 将大地坐标系转换为UTM下的局部坐标系ENU
  double x = ins->position().lon();
  double y = ins->position().lat();
  x *= DEG_TO_RAD_LOCAL;
  y *= DEG_TO_RAD_LOCAL;

  pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(ins->position().height());

  // 2. orientation
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
  gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
  gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());

  gps_writer_->Write(gps);
  if (config_.tf().enable()) {
    TransformStamped transform;
    GpsToTransformStamped(gps, &transform);
    tf_broadcaster_.SendTransform(transform);
  }
}

/**
 * @brief 根据融合了GNSS的惯性导航结果，进行一定处理后填充CorrectedImu，并发布至相关话题
 * @details 将加速度和线加速度从FLU坐标系转换为ENU坐标系
 * 
 * @param message 
 */
void DataParser::PublishCorrimu(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  auto imu = std::make_shared<CorrectedImu>();
  double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
  imu->mutable_header()->set_timestamp_sec(unix_sec);

  auto *imu_msg = imu->mutable_imu();
  imu_msg->mutable_linear_acceleration()->set_x(
      -ins->linear_acceleration().y());
  imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().x());
  imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());

  imu_msg->mutable_angular_velocity()->set_x(-ins->angular_velocity().y());
  imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().x());
  imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());

  imu_msg->mutable_euler_angles()->set_x(ins->euler_angles().x());
  imu_msg->mutable_euler_angles()->set_y(-ins->euler_angles().y());
  imu_msg->mutable_euler_angles()->set_z(ins->euler_angles().z() -
                                         90 * DEG_TO_RAD_LOCAL);

  corrimu_writer_->Write(imu);
}

void DataParser::PublishEphemeris(const MessagePtr message) {
  auto eph = std::make_shared<GnssEphemeris>(*As<GnssEphemeris>(message));
  gnssephemeris_writer_->Write(eph);
}

void DataParser::PublishObservation(const MessagePtr message) {
  auto observation =
      std::make_shared<EpochObservation>(*As<EpochObservation>(message));
  epochobservation_writer_->Write(observation);
}

void DataParser::PublishHeading(const MessagePtr message) {
  auto heading = std::make_shared<Heading>(*As<Heading>(message));
  heading_writer_->Write(heading);
}

void DataParser::GpsToTransformStamped(const std::shared_ptr<Gps> &gps,
                                       TransformStamped *transform) {
  transform->mutable_header()->set_timestamp_sec(gps->header().timestamp_sec());
  transform->mutable_header()->set_frame_id(config_.tf().frame_id());
  transform->set_child_frame_id(config_.tf().child_frame_id());
  auto translation = transform->mutable_transform()->mutable_translation();
  translation->set_x(gps->localization().position().x());
  translation->set_y(gps->localization().position().y());
  translation->set_z(gps->localization().position().z());
  auto rotation = transform->mutable_transform()->mutable_rotation();
  rotation->set_qx(gps->localization().orientation().qx());
  rotation->set_qy(gps->localization().orientation().qy());
  rotation->set_qz(gps->localization().orientation().qz());
  rotation->set_qw(gps->localization().orientation().qw());
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
