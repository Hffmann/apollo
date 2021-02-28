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

#ifndef CYBER_TOOLS_CYBER_CARLA_PLAYER_PLAY_TASK_H_
#define CYBER_TOOLS_CYBER_CARLA_PLAYER_PLAY_TASK_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "cyber/message/raw_message.h"
#include "cyber/node/writer.h"

//      TEST INIT START

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/transform/proto/transform.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"


//      TEST INIT END

namespace apollo {
namespace cyber {
namespace record {

class PlayTask {
 public:
  using MessagePtr = std::shared_ptr<message::RawMessage>;
  using WriterPtr = std::shared_ptr<Writer<message::RawMessage>>;
  using ChassisPtr = std::shared_ptr<apollo::canbus::Chassis>;
  using ChassisWriterPtr = std::shared_ptr<Writer<apollo::canbus::Chassis>>;
  using PadMessagePtr = std::shared_ptr<apollo::control::PadMessage>;
  using PadMessageWriterPtr = std::shared_ptr<Writer<apollo::control::PadMessage>>;
  using LocalizationEstimatePtr = std::shared_ptr<apollo::localization::LocalizationEstimate>;
  using LocalizationEstimateWriterPtr = std::shared_ptr<Writer<apollo::localization::LocalizationEstimate>>;
  using TransformStampedsPtr = std::shared_ptr<apollo::transform::TransformStampeds>;
  using TransformStampedsWriterPtr = std::shared_ptr<Writer<apollo::transform::TransformStampeds>>;
  using PointCloudPtr = std::shared_ptr<apollo::drivers::PointCloud>;
  using PointCloudWriterPtr = std::shared_ptr<Writer<apollo::drivers::PointCloud>>;
  using CompressedImagePtr = std::shared_ptr<apollo::drivers::CompressedImage>;
  using CompressedImageWriterPtr = std::shared_ptr<Writer<apollo::drivers::CompressedImage>>;
  using PerceptionObstaclesPtr = std::shared_ptr<apollo::perception::PerceptionObstacles>;
  using PerceptionObstaclesWriterPtr = std::shared_ptr<Writer<apollo::perception::PerceptionObstacles>>;
  using ADCTrajectoryPtr = std::shared_ptr<apollo::planning::ADCTrajectory>;
  using ADCTrajectoryWriterPtr = std::shared_ptr<Writer<apollo::planning::ADCTrajectory>>;


  PlayTask(const MessagePtr& msg, const WriterPtr& writer,
                   uint64_t msg_real_time_ns);

  PlayTask(const ChassisPtr& chassis, const ChassisWriterPtr& chassis_writer,
                  uint64_t msg_real_time_ns);

  PlayTask(const PadMessagePtr& pad_msg,
           const PadMessageWriterPtr& pad_msg_writer,
                   uint64_t msg_real_time_ns);

  PlayTask(const LocalizationEstimatePtr& localization,
           const LocalizationEstimateWriterPtr& localization_writer,
                   uint64_t msg_real_time_ns);

  PlayTask(const TransformStampedsPtr& tf,
          const TransformStampedsWriterPtr& tf_writer,
                  uint64_t msg_real_time_ns);

  PlayTask(const PointCloudPtr& pointcloud,
           const PointCloudWriterPtr& pointcloud_writer,
                   uint64_t msg_real_time_ns);

  PlayTask(const CompressedImagePtr& compressedimage,
           const CompressedImageWriterPtr& compressedimage_writer,
                   uint64_t msg_real_time_ns);

  PlayTask(const PerceptionObstaclesPtr& perceptionobstacles,
           const PerceptionObstaclesWriterPtr& perceptionobstacles_writer,
                   uint64_t msg_real_time_ns);

  PlayTask(const ADCTrajectoryPtr& trajectory,
           const ADCTrajectoryWriterPtr& trajectory_writer,
                   uint64_t msg_real_time_ns);

  virtual ~PlayTask() {}

  void Play();

  uint64_t msg_real_time_ns() const { return msg_real_time_ns_; }
  static uint64_t played_msg_num() { return played_msg_num_.load(); }

 private:
  MessagePtr msg_;
  WriterPtr writer_;
  ChassisPtr chassis_;
  ChassisWriterPtr chassis_writer_;
  PadMessagePtr pad_msg_;
  PadMessageWriterPtr pad_msg_writer_;
  LocalizationEstimatePtr localization_;
  LocalizationEstimateWriterPtr localization_writer_;
  TransformStampedsPtr tf_;
  TransformStampedsWriterPtr tf_writer_;
  PointCloudPtr pointcloud_;
  PointCloudWriterPtr pointcloud_writer_;
  CompressedImagePtr compressedimage_;
  CompressedImageWriterPtr compressedimage_writer_;
  PerceptionObstaclesPtr perceptionobstacles_;
  PerceptionObstaclesWriterPtr perceptionobstacles_writer_;
  ADCTrajectoryPtr trajectory_;
  ADCTrajectoryWriterPtr trajectory_writer_;
  uint64_t msg_real_time_ns_;

  static std::atomic<uint64_t> played_msg_num_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_H_
