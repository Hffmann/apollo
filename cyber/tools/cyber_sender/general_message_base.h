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

#ifndef TOOLS_CVT_SENDER_GENERAL_MESSAGE_BASE_H_
#define TOOLS_CVT_SENDER_GENERAL_MESSAGE_BASE_H_

#include <map>
#include <utility>

#include "cyber/cyber.h"
#include "cyber/tools/cyber_sender/renderable_message.h"

class Screen;

class GeneralMessageBase : public RenderableMessage {
 protected:
  static void SendMessage(GeneralMessageBase* baseMsg,
                           const google::protobuf::Message& msg,
                           int* jump_lines, const Screen* s, int* line_no,
                           int indent);
  static void SendField(GeneralMessageBase* baseMsg,
                         const google::protobuf::Message& msg, int* jump_lines,
                         const Screen* s, int* line_no, int indent,
                         const google::protobuf::Reflection* ref,
                         const google::protobuf::FieldDescriptor* field,
                         int index);

  static int LineCount(const google::protobuf::Message& msg, int screen_width);
  static int LineCountOfField(const google::protobuf::Message& msg,
                              int screen_width,
                              const google::protobuf::FieldDescriptor* field,
                              const google::protobuf::Reflection* reflection,
                              bool is_folded = true);

  void insertRepeatedMessage(int line_no, GeneralMessageBase* item) {
    children_map_.insert(std::make_pair(line_no, item));
  }

  std::stringstream& getStream() { return ss_; }

  std::string& getNextFieldName() { return nextFieldName_; }

  bool& getIsSenderField() { return isSenderField_; }

  void setNextFieldName(std::string nextFieldName, bool setPrev) {
    nextFieldName_ = nextFieldName;
    if (setPrev) {
      prevFieldName_ = nextFieldName;
    }
  }

  void setNextFieldName() {
    nextFieldName_ = prevFieldName_;
  }

  void isSenderField() {
    isSenderField_ = !isSenderField_;
  }

  std::string insertString(std::string str) {
    if (isSenderField_) {
      ss_ << str << "\n";
    }
    return str;
  }

  RenderableMessage* Child(int line_no) const override;

  explicit GeneralMessageBase(RenderableMessage* parent = nullptr)
      : RenderableMessage(parent),
        children_map_(),
        ss_(),
        prevFieldName_(),
        nextFieldName_(),
        isSenderField_(false) {}

  ~GeneralMessageBase(void) { clear(); }

  void clear(void) {
    for (auto& iter : children_map_) {
      delete iter.second;
    }
    children_map_.clear();

    ss_.str("");
    ss_.clear();

    prevFieldName_.clear();
    nextFieldName_.clear();
  }

  GeneralMessageBase(const GeneralMessageBase&) = delete;
  GeneralMessageBase& operator=(const GeneralMessageBase&) = delete;

  std::map<const int, GeneralMessageBase*> children_map_;
  std::stringstream ss_;
  std::string prevFieldName_;
  std::string nextFieldName_;
  bool isSenderField_;
};

#endif  // TOOLS_CVT_MONITOR_GENERAL_MESSAGE_BASE_H_
