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

#ifndef CYBER_TOOLS_CYBER_CARLA_PLAYER_PLAY_TASK_SENDER_H_
#define CYBER_TOOLS_CYBER_CARLA_PLAYER_PLAY_TASK_SENDER_H_

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"

// #include "zmq.hpp"
// #include <time.h>
// #include <sys/time.h>
// #include <iostream>

namespace apollo {
namespace cyber {
namespace record {

class PlayTaskSender {
 public:
  PlayTaskSender();

  ~PlayTaskSender() {
    channel_node_.reset();
    channel_reader_.reset();
    channel_message_.reset();
    if (raw_msg_class_) {
      delete raw_msg_class_;
      raw_msg_class_ = nullptr;
    }
  }

  void Start(std::string channel_name);

 private:

  static void PrintMessage(const google::protobuf::Message& msg,
                           int indent);

  static void PrintField(const google::protobuf::Message& msg,
                         int indent, const google::protobuf::Reflection* ref,
                         const google::protobuf::FieldDescriptor* field,
                         int index);

  void UpdateRawMessage(
      const std::shared_ptr<apollo::cyber::message::RawMessage>& raw_msg) {
    set_has_message_come(true);
//     msg_time_ = apollo::cyber::Time::MonoTime();
//     ++frame_counter_;
    std::lock_guard<std::mutex> _g(inner_lock_);
    channel_message_.reset();
    channel_message_ = raw_msg;
  }

  std::shared_ptr<apollo::cyber::message::RawMessage> CopyMsgPtr(void) const {
    decltype(channel_message_) channel_msg;
    {
      std::lock_guard<std::mutex> g(inner_lock_);
      channel_msg = channel_message_;
    }
    return channel_msg;
  }

  void set_message_type(const std::string& msgTypeName) {
    message_type_ = msgTypeName;
  }
  const std::string& message_type(void) const { return message_type_; }

//   bool is_enabled(void) const { return channel_reader_ != nullptr; }
  bool has_message_come(void) const { return has_message_come_; }

  const std::string& NodeName(void) const { return node_name_; }

  std::string GetChannelName(void) const {
    return channel_reader_->GetChannelName();
  }

  void set_has_message_come(bool b) { has_message_come_ = b; }

  bool has_message_come_;
  std::string message_type_;
//   apollo::cyber::Time msg_time_;

  std::unique_ptr<apollo::cyber::Node> channel_node_;

  std::string node_name_;

  std::shared_ptr<apollo::cyber::message::RawMessage> channel_message_;
  std::shared_ptr<apollo::cyber::Reader<apollo::cyber::message::RawMessage>>
      channel_reader_;
  mutable std::mutex inner_lock_;

  google::protobuf::Message* raw_msg_class_;

};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_SENDER_H_
