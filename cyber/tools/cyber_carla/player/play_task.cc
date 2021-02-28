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

#include "cyber/tools/cyber_carla/player/play_task.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace record {

std::atomic<uint64_t> PlayTask::played_msg_num_ = {0};

// RAW MESSAGE

PlayTask::PlayTask(const MessagePtr& msg, const WriterPtr& writer,
                   uint64_t msg_real_time_ns)
    : msg_(msg),
      writer_(writer) {}

// CHASSIS

PlayTask::PlayTask(
                   const ChassisPtr& chassis,
                   const ChassisWriterPtr& chassis_writer,
                   uint64_t msg_real_time_ns)
    : chassis_(chassis),
      chassis_writer_(chassis_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// PAD MESSAGE

PlayTask::PlayTask(
                   const PadMessagePtr& pad_msg,
                   const PadMessageWriterPtr& pad_msg_writer,
                   uint64_t msg_real_time_ns)
    : pad_msg_(pad_msg),
      pad_msg_writer_(pad_msg_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// LOCALIZATION ESTIMATE

PlayTask::PlayTask(const LocalizationEstimatePtr& localization,
                   const LocalizationEstimateWriterPtr& localization_writer,
                   uint64_t msg_real_time_ns)
    : localization_(localization),
      localization_writer_(localization_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// TRANSFORM

PlayTask::PlayTask(const TransformStampedsPtr& tf,
                   const TransformStampedsWriterPtr& tf_writer,
                   uint64_t msg_real_time_ns)
    : tf_(tf),
      tf_writer_(tf_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// POINT CLOUD (LIDAR)

PlayTask::PlayTask(const PointCloudPtr& pointcloud,
                   const PointCloudWriterPtr& pointcloud_writer,
                   uint64_t msg_real_time_ns)
    : pointcloud_(pointcloud),
      pointcloud_writer_(pointcloud_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// COMPRESSED IMAGE (CAMERA)

PlayTask::PlayTask(const CompressedImagePtr& compressedimage,
                   const CompressedImageWriterPtr& compressedimage_writer,
                   uint64_t msg_real_time_ns)
    : compressedimage_(compressedimage),
      compressedimage_writer_(compressedimage_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// PERCEPTION OBSTACLES (GROUND TRUTH)

PlayTask::PlayTask(const PerceptionObstaclesPtr& perceptionobstacles,
                   const PerceptionObstaclesWriterPtr& perceptionobstacles_writer,
                   uint64_t msg_real_time_ns)
    : perceptionobstacles_(perceptionobstacles),
      perceptionobstacles_writer_(perceptionobstacles_writer),
      msg_real_time_ns_(msg_real_time_ns){}

// ADC TRAJECTORY

PlayTask::PlayTask(const ADCTrajectoryPtr& trajectory,
                   const ADCTrajectoryWriterPtr& trajectory_writer,
                   uint64_t msg_real_time_ns)
    : trajectory_(trajectory),
      trajectory_writer_(trajectory_writer),
      msg_real_time_ns_(msg_real_time_ns){}


void PlayTask::Play() {

  if (writer_ == nullptr &&
      chassis_writer_ == nullptr &&
      pad_msg_writer_ == nullptr &&
      localization_writer_ == nullptr &&
      tf_writer_ == nullptr &&
      pointcloud_writer_ == nullptr &&
      compressedimage_writer_ == nullptr &&
      perceptionobstacles_writer_ == nullptr &&
      trajectory_writer_ == nullptr){
    AERROR << "writer is nullptr, can't write message.";
    return;
  }

  if (chassis_ != nullptr){
    if (!chassis_writer_->Write(chassis_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (pad_msg_ != nullptr){
    if (!pad_msg_writer_->Write(pad_msg_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (localization_ != nullptr){
    if (!localization_writer_->Write(localization_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (tf_ != nullptr){
    if (!tf_writer_->Write(tf_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (pointcloud_ != nullptr){
    if (!pointcloud_writer_->Write(pointcloud_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (compressedimage_ != nullptr){
    if (!compressedimage_writer_->Write(compressedimage_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (perceptionobstacles_ != nullptr){
    if (!perceptionobstacles_writer_->Write(perceptionobstacles_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  if (trajectory_ != nullptr){
    if (!trajectory_writer_->Write(trajectory_)) {
      AERROR << "write message failed, played num: " << played_msg_num_.load()
            << ", real time: " << msg_real_time_ns_;
      return;
    }
  }

  played_msg_num_.fetch_add(1);

  ADEBUG << "write message succ, played num: " << played_msg_num_.load()
         << ", real time: " << msg_real_time_ns_;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
