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

#include "cyber/tools/cyber_carla/player/play_task_sender.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// #include "cyber/init.h"
// #include "cyber/service_discovery/topology_manager.h"
// #include "cyber/tools/cyber_monitor/cyber_topology_message.h"
// #include "cyber/tools/cyber_monitor/renderable_message.h"

//      TEST INIT END

namespace apollo {
namespace cyber {
namespace record {

constexpr int INT_FLOAT_PRECISION = 6;
constexpr int DOULBE_PRECISION = 9;

//      TEST FILE DEF START

PlayTaskSender::PlayTaskSender()
      : has_message_come_(false),
        message_type_(),
        channel_node_(nullptr),
        node_name_(),
        channel_message_(nullptr),
        channel_reader_(nullptr),
        inner_lock_(),
        raw_msg_class_(nullptr) {}

void PlayTaskSender::Start(std::string channel_name) {

//   if (channel_name.empty() || node_name_.empty()) {
//     return CastErrorCode2Ptr(ErrorCode::ChannelNameOrNodeNameIsEmpty);
//   }
//   if (channel_node_ != nullptr || channel_reader_ != nullptr) {
//     return CastErrorCode2Ptr(ErrorCode::NoCloseChannel);
//   }

//   CyberTopologyMessage topology_msg(channel_name);
//
//   auto topology_callback =
//       [&topology_msg](const apollo::cyber::proto::ChangeMsg &change_msg) {
//         topology_msg.TopologyChanged(change_msg);
//       };

  auto channel_manager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->channel_manager();

  auto node_manager =
      apollo::cyber::service_discovery::TopologyManager::Instance()
          ->node_manager();

//   channel_manager->AddChangeListener(topology_callback);

//   const std::string& channelName = changeMsg.role_attr().channel_name();

  std::vector<apollo::cyber::RoleAttributes> role_vec;
  channel_manager->GetWriters(&role_vec);

  std::vector<apollo::cyber::proto::RoleAttributes> node_vec;
  node_manager->GetNodes(&node_vec);

  std::string msgdata;

//   for (auto& node_attr : node_vec) {
//
//     node_attr.SerializeToString(&msgdata);
//
//     std::cout << "\n\n\n\n" << "node name: " << msgdata << "\n\n\n\n" << std::endl;
//
// //     std::vector<apollo::cyber::RoleAttributes> readers_vec;
// //     channel_manager->GetReadersOfNode(node_attr.node_name(), &readers_vec);
//
//     if (node_attr.node_name() == channel_name) {
//
//         node_attr.SerializeToString(&msgdata);
//         std::cout << "\n\n\n\n" << "channel name: " << channel_name << "\n\n\n\n" << std::endl;
//
//     }
//   }


//
//   std::cout << "\n\n\n\n" << "NODE: " << typeid(node_vec).name() << "\n\n\n\n" << std::endl;

//   proto::RoleAttributes role;
  for (auto &role : role_vec) {
//   const std::string& msgTypeName = role.message_type();

    if (role.channel_name() == channel_name) {



//       apollo::cyber::proto::RoleAttributes specific_role = role;
//
//       specific_role.set_process_id(12345);

//       std::cout << "\n\n\n" << specific_role.process_id() << "\n\n\n" << std::endl;

//       specific_role.set_process_id(process_id);

      const std::string& msgTypeName = role.message_type();
      set_message_type(msgTypeName);

      node_name_ = "cyber_sender_play_" + role.node_name();
      channel_node_ = apollo::cyber::CreateNode(node_name_);

//       std::cout << "\n\n\n\n" << typeid(channel_node_).name() << "\n\n\n\n" << std::endl;

//       if (channel_node_ == nullptr) {
//         std::cout << "channel_node_" << "\n" << std::endl;
//       }

//       std::cout << has_message_come_ << "\n" << std::endl;

      auto callback =
      [this](
          const std::shared_ptr<apollo::cyber::message::RawMessage>& raw_msg) {
        UpdateRawMessage(raw_msg);
      };

//       const std::shared_ptr<apollo::cyber::message::RawMessage>& raw_msg = nullptr;
//       UpdateRawMessage(raw_msg);

//       auto raw_msg = std::make_shared<message::RawMessage>(itr->content);
//
//       UpdateRawMessage(raw_msg);

//       std::cout << has_message_come_ << "\n" << std::endl;
//       if (channel_message_ == nullptr) {
//         std::cout << "channel_message_" << "\n" << std::endl;
//       }

      channel_reader_ = channel_node_->CreateReader<apollo::cyber::message::RawMessage>(channel_name, callback);
//       std::cout << typeid(role.channel_name()).name() << "\n";

//       if (channel_reader_ == nullptr) {
//         std::cout << "channel_reader_" << "\n" << std::endl;
//       }
//       if (has_message_come()) {
//     if (raw_msg_class_ == nullptr) {
      channel_reader_->Observe();
      const auto &channel_msg = channel_reader_->GetLatestObserved();

      if (channel_msg == nullptr) {
        AERROR << "channel_msg is not ready!";
      }

      std::cout << "HasReceived: " << channel_reader_->HasReceived()<< "\n" << std::endl;
      std::cout << "Empty: " << channel_reader_->Empty() << "\n" << std::endl;
      std::cout << "GetDelaySec: " << channel_reader_->GetDelaySec() << "\n" << std::endl;

//
//       auto latest = channel_reader_->GetLatestObserved();

//       std::cout << "HAS RECEIVED: " << latest << "\n" << std::endl;

//       std::cout << "\n\n\n\n" << channel_name << " " << role.channel_name() << "\n\n\n\n" << std::endl;

      if (has_message_come()) {

        if (raw_msg_class_ == nullptr) {
          auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
          raw_msg_class_ = rawFactory->GenerateMessageByType(message_type());
        }

        if (raw_msg_class_ == nullptr) {
          std::cout << "Cannot Generate Message by Message Type" << std::endl;
        }


    //     }

    //     if (raw_msg_class == nullptr) {
    //       std::cout << "Cannot Generate Message by Message Type" << "\n" << std::endl;
    // //       s->AddStr(0, (*line_no)++, "Cannot Generate Message by Message Type");
    //     }

  //       std::cout << "ChannelName: " << channel_reader_->GetChannelName().c_str() << "\n" << std::endl;
  //
  //       std::cout << "MessageType: " << message_type() << "\n" << std::endl;

        decltype(channel_message_) channel_msg = CopyMsgPtr();

//         if (channel_msg->message.size()) {
//           std::cout << channel_msg->message.size() << "\n" << std::endl;
//         }
    //   if (channel_reader_ == nullptr) {
    //     channel_node_.reset();
    //     return CastErrorCode2Ptr(ErrorCode::CreateReaderFailed);
    //   }
    //   return this;

//         std::cout << channel_msg->message << "\n" << std::endl;

  //       raw_msg_class_->ParseFromString(channel_msg->message);

//         std::cout << "\n\n\n\n" << "ENTROU" << "\n\n\n\n" << std::endl;

        if (raw_msg_class_->ParseFromString(channel_msg->message)) {


    //     int lcount = LineCount(*raw_msg_class_, s->Width());
    //     page_item_count_ = s->Height() - *line_no;
    //     pages_ = lcount / page_item_count_ + 1;
    //     SplitPages(key);
    //     int jump_lines = page_index_ * page_item_count_;
    //     jump_lines <<= 2;
    //     jump_lines /= 5;
          PrintMessage(*raw_msg_class_, 0);
        }
      }
//       }
    }
  }

//   std::cout << "\n\n\n" << msgTypeName << "\n\n\n";

//   set_message_type(msgTypeName);


//   auto channel_node = apollo::cyber::CreateNode(node_name);
//   if (channel_node_ == nullptr) {
//     return CastErrorCode2Ptr(ErrorCode::CreateNodeFailed);
//   }

//   auto callback =
//       [this](
//           const std::shared_ptr<apollo::cyber::message::RawMessage>& raw_msg) {
//         UpdateRawMessage(raw_msg);
//       };
//
//   auto channel_reader = channel_node->CreateReader<apollo::cyber::message::RawMessage>(
//           channel_name, callback);







}



void PlayTaskSender::PrintMessage(const google::protobuf::Message& msg, int indent) {
  const google::protobuf::Reflection* reflection = msg.GetReflection();
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  if (descriptor->options().map_entry()) {
    fields.push_back(descriptor->field(0));
    fields.push_back(descriptor->field(1));
  } else {
    reflection->ListFields(msg, &fields);
  }
  for (std::size_t i = 0; i < fields.size(); ++i) {
//     if (*line_no > s->Height()) {
//       break;
//     }
    const google::protobuf::FieldDescriptor* field = fields[i];
//     if (field->is_repeated()) {
//       if (*jump_lines) {
//         --(*jump_lines);
//       } else {
//         std::ostringstream out_str;
//         const std::string& fieldName = field->name();
//         out_str << fieldName << ": ";
//         out_str << "+[" << reflection->FieldSize(msg, field) << " items]";
// //         GeneralMessage* item =
// //             new GeneralMessage(baseMsg, &msg, reflection, field);
// //         if (item) {
// //           baseMsg->insertRepeatedMessage(*line_no, item);
// //         }
//         s->AddStr(indent, (*line_no)++, out_str.str().c_str());
//       }
//     } else {
//       PrintField(baseMsg, msg, jump_lines, s, line_no, indent, reflection,
//                  field, -1);
//     }  // end else

    PrintField(msg, indent, reflection, field, -1);

  }    // end for

//   const google::protobuf::UnknownFieldSet& unknown_fields =
//       reflection->GetUnknownFields(msg);
//   if (!unknown_fields.empty()) {
//     Screen::ColorPair c = s->Color();
//     s->ClearCurrentColor();
//     s->SetCurrentColor(Screen::RED_BLACK);
//     s->AddStr(indent, (*line_no)++, "Have Unknown Fields");
//     s->ClearCurrentColor();
//     s->SetCurrentColor(c);
//   }
}






void PlayTaskSender::PrintField(
    const google::protobuf::Message& msg,
    int indent, const google::protobuf::Reflection* ref,
    const google::protobuf::FieldDescriptor* field, int index) {
  std::ostringstream out_str;
  std::ios_base::fmtflags old_flags;

  switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD, PRECISION)                                  \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:                      \
    {                                                                             \
      /*if (*jump_lines) {*/                                                      \
      /*--(*jump_lines);*/                                                        \
      /*} else {*/                                                                \
      const std::string& fieldName = field->name();                               \
      out_str << fieldName << ": ";                                               \
      if (field->is_repeated()) {                                                 \
        out_str << "[" << index << "] ";                                          \
      }                                                                           \
      old_flags = out_str.flags();                                                \
      out_str << std::fixed << std::setprecision(PRECISION)                       \
              << (field->is_repeated()                                            \
                      ? ref->GetRepeated##METHOD(msg, field, index)               \
                      : ref->Get##METHOD(msg, field));                            \
      out_str << "\n";                                                            \
      out_str.flags(old_flags);                                                   \
      std::cout << std::string(indent, ' ') << out_str.str().c_str() << std::endl;\
        /*s->AddStr(indent, (*line_no)++, out_str.str().c_str());*/               \
    }                                                                             \
    break

    OUTPUT_FIELD(INT32, Int32, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(INT64, Int64, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(UINT32, UInt32, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(UINT64, UInt64, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(FLOAT, Float, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(DOUBLE, Double, DOULBE_PRECISION);
    OUTPUT_FIELD(BOOL, Bool, INT_FLOAT_PRECISION);
#undef OUTPUT_FIELD

    case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
      std::string scratch;
      const std::string& str =
          field->is_repeated()
              ? ref->GetRepeatedStringReference(msg, field, index, &scratch)
              : ref->GetStringReference(msg, field, &scratch);
      {
//         int lineWidth = 0;
        std::size_t i = 0;

        for (; i < str.size() /*&& *jump_lines > 0*/; ++i) {
          if (str[i] == '\n' || str[i] == '\r') {
//             --(*jump_lines);
//             lineWidth = 0;
            out_str << "\n";
          } /*else {
            ++lineWidth;
            if (lineWidth == s->Width()) {
              --(*jump_lines);
              lineWidth = 0;
            }
          }*/
        }

//         if (*jump_lines == 0) {
//         lineWidth = 0;
//         unsigned line_count = 1;

        const std::string& fieldName = field->name();
        out_str << fieldName << ": ";
        if (field->is_repeated()) {
          out_str << "[" << index << "] ";
        }

        for (; i < str.size(); ++i) {
          char ch = str[i];
          if (str[i] == '\n' || str[i] == '\r') {
//             ++line_count;
//             lineWidth = 0;
            ch = '\n';
          } /*else {
            ++lineWidth;
            if (lineWidth == s->Width()) {
              ++line_count;
              lineWidth = 0;
            }
          }*/
          out_str << ch;
        }
        out_str << "\n";
        std::cout << std::string(indent, ' ') << out_str.str().c_str() << std::endl;
//         s->AddStr(indent, *line_no, out_str.str().c_str());
//         (*line_no) += line_count;
      }
//      }

      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
//       if (*jump_lines) {
//         --(*jump_lines);
//       } else {
      {
        const std::string& fieldName = field->name();
        out_str << fieldName << ": ";
        if (field->is_repeated()) {
          out_str << "[" << index << "] ";
        }
        int enum_value = field->is_repeated()
                              ? ref->GetRepeatedEnumValue(msg, field, index)
                              : ref->GetEnumValue(msg, field);
        const google::protobuf::EnumValueDescriptor* enum_desc =
            field->enum_type()->FindValueByNumber(enum_value);
        if (enum_desc != nullptr) {
          out_str << enum_desc->name();
        } else {
          out_str << enum_value;
        }
        out_str << "\n";
        std::cout << std::string(indent, ' ') << out_str.str().c_str() << std::endl;
  //       s->AddStr(indent, (*line_no)++, out_str.str().c_str());
  //       }
      }
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
//       if (!*jump_lines) {
      {
        const std::string& fieldName = field->name();
        out_str << fieldName;
        if (!field->is_map()) {
          out_str << ": ";
          if (field->is_repeated()) {
            out_str << "[" << index << "] ";
          }
        }
        out_str << "\n";
        std::cout << out_str.str().c_str() << std::endl;
  //       s->AddStr(indent, (*line_no)++, out_str.str().c_str());
  //       } else {
  //         --(*jump_lines);
  //       }
        PlayTaskSender::PrintMessage(
            field->is_repeated() ? ref->GetRepeatedMessage(msg, field, index)
                                : ref->GetMessage(msg, field), indent + 2);
      }
      break;
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
