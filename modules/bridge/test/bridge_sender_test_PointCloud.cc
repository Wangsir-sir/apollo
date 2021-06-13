/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <thread>

#include "modules/drivers/proto/pointcloud.pb.h"


#include "cyber/common/log.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/time/clock.h"
#include "modules/bridge/common/bridge_proto_serialized_buf.h"

using apollo::cyber::Clock;

bool send(const std::string &remote_ip, uint16_t remote_port, uint32_t count) {
  if (count == 0) {
    count = 10000;
  }

  for (uint32_t i = 0; i < count; i++) {
    double timestamp_ = Clock::NowInSeconds() + 2.0;
    auto pb_msg = std::make_shared<apollo::drivers::PointCloud>();
    pb_msg->mutable_header()->set_sequence_num(i);
    pb_msg->mutable_header()->set_timestamp_sec(timestamp_);
    pb_msg->set_frame_id ("lslidar_frame");
    pb_msg->set_height(10);
    pb_msg->set_is_dense(true);
    pb_msg->set_measurement_time(10.0);
    pb_msg->set_width(10);
    for(int i =  0;i < 100;i++){
      auto point = pb_msg->add_point();
      point->set_x(i);
      point->set_y(i);
      point->set_z(i);
      point->set_intensity(i);
      point->set_timestamp(i);
    } 

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(remote_port);

    ADEBUG << "connecting to server... ";

    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

    int res =
        connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (res < 0) {
      ADEBUG << "connected server failed ";
      continue;
    }

    ADEBUG << "connected to server success. port [" << remote_port << "]";

    apollo::bridge::BridgeProtoSerializedBuf<apollo::drivers::PointCloud> proto_buf;
    proto_buf.Serialize(pb_msg, "PointCloud");
    // AINFO  <<  "Count:"<<proto_buf.GetSerializedBufCount();
    // AINFO << "Size 0 :" << proto_buf.GetSerializedBufSize(0) << " | 1: " << proto_buf.GetSerializedBufSize(1);
    AWARN ;
    for (size_t j = 0; j < proto_buf.GetSerializedBufCount(); j++) {
      ssize_t nbytes = send(sock_fd, proto_buf.GetSerializedBuf(j),
                            proto_buf.GetSerializedBufSize(j), 0);
      if (nbytes != static_cast<ssize_t>(proto_buf.GetSerializedBufSize(j))) {
        AWARN << "sent msg failed ";
        break;
      }
      AWARN << "total frames: " << proto_buf.GetSerializedBufCount() << " Epoch:"<< i << " "<<"sent " << nbytes << " bytes  frame " << j;
      // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    close(sock_fd);

    // 1000Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  return true;
}

int main(int argc, char *argv[]) {
  uint32_t count = 10000;

  uint16_t port = (uint16_t) atoi(argv[1]);

  send("127.0.0.1", port, count);
  return 0;
}
