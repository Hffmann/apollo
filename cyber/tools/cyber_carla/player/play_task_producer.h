/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_TOOLS_CYBER_CARLA_PLAYER_PLAY_TASK_PRODUCER_H_
#define CYBER_TOOLS_CYBER_CARLA_PLAYER_PLAY_TASK_PRODUCER_H_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/message/raw_message.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"
#include "cyber/record/record_reader.h"
#include "cyber/tools/cyber_carla/player/play_param.h"
#include "cyber/tools/cyber_carla/player/play_task_buffer.h"

//      TEST INIT START

// #include "modules/canbus/proto/chassis.pb.h"

//      TEST INIT

#include "zmq.hpp"
#include <time.h>
#include <sys/time.h>
#include <iostream>

namespace apollo {
namespace cyber {
namespace record {

class PlayTaskProducer {
 public:
  using NodePtr = std::shared_ptr<Node>;
  using ThreadPtr = std::unique_ptr<std::thread>;
  using TaskBufferPtr = std::shared_ptr<PlayTaskBuffer>;
  using RecordReaderPtr = std::shared_ptr<RecordReader>;
  using ChassisWriterPtr = std::shared_ptr<Writer<apollo::canbus::Chassis>>;
  using PadMessageWriterPtr = std::shared_ptr<Writer<apollo::control::PadMessage>>;
  using LocalizationEstimateWriterPtr = std::shared_ptr<Writer<apollo::localization::LocalizationEstimate>>;
  using TransformStampedsWriterPtr = std::shared_ptr<Writer<apollo::transform::TransformStampeds>>;
  using PointCloudWriterPtr = std::shared_ptr<Writer<apollo::drivers::PointCloud>>;
  using CompressedImageWriterPtr = std::shared_ptr<Writer<apollo::drivers::CompressedImage>>;
  using PerceptionObstaclesWriterPtr = std::shared_ptr<Writer<apollo::perception::PerceptionObstacles>>;
  using ADCTrajectoryWriterPtr = std::shared_ptr<Writer<apollo::planning::ADCTrajectory>>;

//   using WriterMap = std::unordered_map<std::string, WriterPtr>;

  using MessageTypeMap = std::unordered_map<std::string, std::string>;

  PlayTaskProducer(const TaskBufferPtr& task_buffer,
                   const PlayParam& play_param);
  virtual ~PlayTaskProducer();

  bool Init();
  void Start(std::string message);
  void Stop();

  const PlayParam& play_param() const { return play_param_; }
  bool is_stopped() const { return is_stopped_.load(); }

 private:
  bool ReadRecordInfo();
  bool UpdatePlayParam();
  bool CreateWriters();
  void ThreadFunc();

  PlayParam play_param_;
  TaskBufferPtr task_buffer_;
  ThreadPtr produce_th_;

  std::atomic<bool> is_initialized_;
  std::atomic<bool> is_stopped_;

  NodePtr node_;

  ChassisWriterPtr chassis_writer_;
  PadMessageWriterPtr pad_msg_writer_;
  LocalizationEstimateWriterPtr localization_writer_;
  TransformStampedsWriterPtr tf_writer_;
  PointCloudWriterPtr pointcloud_writer_;
  CompressedImageWriterPtr compressedimage_writer_;
  PerceptionObstaclesWriterPtr perceptionobstacles_writer_;
  ADCTrajectoryWriterPtr trajectory_writer_;

//   WriterMap writers_;

  MessageTypeMap msg_types_;
  std::vector<RecordReaderPtr> record_readers_;

  uint64_t earliest_begin_time_;
  uint64_t latest_end_time_;
  uint64_t total_msg_num_;

  static const uint32_t kMinTaskBufferSize;
  static const uint32_t kPreloadTimeSec;
  static const uint64_t kSleepIntervalNanoSec;

  std::string receiverMsg_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_
