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

#include "modules/drivers/canbus/can_client/fake/fake_can_client.h"

#include <cstring>
#include <thread>

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool FakeCanClient::Init(const CANCardParameter &param) { return true; }

ErrorCode FakeCanClient::Start() { return ErrorCode::OK; }

void FakeCanClient::Stop() {}

ErrorCode FakeCanClient::Send(const std::vector<CanFrame> &frames,
                              int32_t *const frame_num) {
  if (frame_num == nullptr) {
    AERROR << "frame_num pointer is null";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  if (static_cast<size_t>(*frame_num) != frames.size()) {
    AERROR << "frame num is incorrect.";
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  for (size_t i = 0; i < frames.size(); ++i) {
    AERROR << "send frame i:" << i;
    AERROR << frames[i].CanFrameString();
    frame_info_ << frames[i].CanFrameString();
  }
  ++send_counter_;
  return ErrorCode::OK;
}

ErrorCode FakeCanClient::Receive(std::vector<CanFrame> *const frames,
                                 int32_t *const frame_num) {
  if (frame_num == nullptr || frames == nullptr) {
    AERROR << "frames or frame_num pointer is null";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  frames->resize(*frame_num);
  const int MOCK_LEN = 8;

  (*frames)[0].id = 0x22C;
  (*frames)[0].len = MOCK_LEN;
  (*frames)[0].data[0] = static_cast<uint8_t>(1);
  (*frames)[0].data[1] = static_cast<uint8_t>(0);
  (*frames)[0].data[2] = static_cast<uint8_t>(0);
  (*frames)[0].data[3] = static_cast<uint8_t>(0);
  (*frames)[0].data[4] = static_cast<uint8_t>(0);
  (*frames)[0].data[5] = static_cast<uint8_t>(0);
  (*frames)[0].data[6] = static_cast<uint8_t>(0);
  (*frames)[0].data[7] = static_cast<uint8_t>(0);

  (*frames)[1].id = 0x204;
  (*frames)[1].len = MOCK_LEN;
  (*frames)[1].data[0] = static_cast<uint8_t>(1);
  (*frames)[1].data[1] = static_cast<uint8_t>(0);
  (*frames)[1].data[2] = static_cast<uint8_t>(0);
  (*frames)[1].data[3] = static_cast<uint8_t>(0);
  (*frames)[1].data[4] = static_cast<uint8_t>(0);
  (*frames)[1].data[5] = static_cast<uint8_t>(0);
  (*frames)[1].data[6] = static_cast<uint8_t>(0);
  (*frames)[1].data[7] = static_cast<uint8_t>(0);

  (*frames)[2].id = 0x200;
  (*frames)[2].len = MOCK_LEN;
  (*frames)[2].data[0] = static_cast<uint8_t>(1);
  (*frames)[2].data[1] = static_cast<uint8_t>(0);
  (*frames)[2].data[2] = static_cast<uint8_t>(0);
  (*frames)[2].data[3] = static_cast<uint8_t>(0);
  (*frames)[2].data[4] = static_cast<uint8_t>(0);
  (*frames)[2].data[5] = static_cast<uint8_t>(0);
  (*frames)[2].data[6] = static_cast<uint8_t>(0);
  (*frames)[2].data[7] = static_cast<uint8_t>(0);


  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ++recv_counter_;
  return ErrorCode::OK;
}

std::string FakeCanClient::GetErrorString(const int32_t /*status*/) {
  return "";
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
