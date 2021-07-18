#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import sys
from modules.map.proto import map_pb2
from modules.map.proto import map_signal_pb2
from modules.map.proto import map_overlap_pb2
from google.protobuf import text_format
from shapely.geometry import LineString, Point

if len(sys.argv) < 3:
    print('Usage: %s [map_file] [signal_file]' % sys.argv[0])
    sys.exit(0)

map_file = sys.argv[1]
signal_file = sys.argv[2]

# 从文件中读取地图消息
with open(map_file, 'r') as fmap:
    map_data = fmap.read()
    map = map_pb2.Map()
    text_format.Parse(map_data, map)

# 从文件中读取信号灯消息
with open(signal_file, 'r') as fsignal:
    signal_data = fsignal.read()
    signal = map_signal_pb2.Signal()
    text_format.Parse(signal_data, signal)

lanes = {} # 地图中ID号和车道中心线LineString组成的字典
lanes_map = {} # 地图中ID号和车道组成的字典
# 遍历地图中每条车道，获得上面两个字典
for lane in map.lane:
    lane_points = []
    lanes_map[lane.id.id] = lane
    # 遍历车道中心线的每一段
    for segment in lane.central_curve.segment:
        for point in segment.line_segment.point:
            lane_points.append((point.x, point.y))
    lane_string = LineString(lane_points)
    lanes[lane.id.id] = lane_string

lines = {}
# 添加车道和信号灯停止线直接的重叠关系
for stop_line in signal.stop_line:
    # 遍历停止线，获取所有点，组成LineString对象
    stop_line_points = []
    for segment in stop_line.segment:
        for point in segment.line_segment.point:
            stop_line_points.append((point.x, point.y))
    stop_line_string = LineString(stop_line_points)
    for lane_id, lane_string in lanes.items():
        # 找到车道中心线和该信号灯停止线的交点，填充重叠信息
        p = stop_line_string.intersection(lane_string)
        if type(p) == Point:
            s = lane_string.project(p)
            overlap = map.overlap.add()
            overlap.id.id = str(lane_id) + "_" + str(signal.id.id)
            obj = overlap.object.add()
            obj.id.id = signal.id.id
            obj.signal_overlap_info.CopyFrom(
                map_overlap_pb2.SignalOverlapInfo())
            obj = overlap.object.add()
            obj.id.id = lane_id
            obj.lane_overlap_info.start_s = s
            obj.lane_overlap_info.end_s = s + 0.1
            obj.lane_overlap_info.is_merge = False

            signal.overlap_id.add().id = overlap.id.id
            lanes_map[lane_id].overlap_id.add().id = overlap.id.id
map.signal.add().CopyFrom(signal)

with open(map_file + "_" + signal_file, 'w') as fmap:
    fmap.write(str(map))
