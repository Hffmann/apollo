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

#include "cyber/tools/cyber_carla/player/play_task_producer.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cstring>
#include <limits>
#include <typeinfo>
#include <fstream>
#include <streambuf>
#include <array>

#include <google/protobuf/message.h>

#include "cyber/common/log.h"
#include "cyber/common/time_conversion.h"
#include "cyber/cyber.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/record/record_viewer.h"

//
//  Task sink in C++
//  Binds PULL socket to tcp://localhost:5558
//  Collects results from workers via that socket
//

// #include <zmq.hpp>
// #include <time.h>
// #include <sys/time.h>


//      TEST INIT START

// #include "gflags/gflags.h"
//
// #include "cyber/common/file.h"
//
// #include "modules/canbus/proto/chassis.pb.h"
// #include "modules/common/adapters/adapter_gflags.h"
// #include "modules/control/common/control_gflags.h"
// #include "modules/control/proto/pad_msg.pb.h"
// #include "modules/localization/proto/localization.pb.h"
// #include "modules/planning/proto/planning.pb.h"

//      TEST INIT END

namespace apollo {
namespace cyber {
namespace record {

//      TEST FILE DEF START

using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::GetProtoFromString;
using apollo::canbus::Chassis;
using apollo::control::PadMessage;
using apollo::localization::LocalizationEstimate;
using apollo::transform::TransformStampeds;
using apollo::planning::ADCTrajectory;
using apollo::drivers::PointCloud;
using apollo::drivers::CompressedImage;
using apollo::perception::PerceptionObstacles;

DEFINE_string(
    chassis_test_file,
    "/apollo/modules/control/testdata/control_tester/chassis.pb.txt",
    "Used for sending simulated Chassis content to the control node.");
DEFINE_string(
    localization_test_file,
    "/apollo/modules/control/testdata/control_tester/localization.pb.txt",
    "Used for sending simulated localization to the control node.");
DEFINE_string(pad_msg_test_file,
              "/apollo/modules/control/testdata/control_tester/pad_msg.pb.txt",
              "Used for sending simulated PadMsg content to the control node.");
DEFINE_string(
    planning_test_file,
    "/apollo/modules/control/testdata/control_tester/planning.pb.txt",
    "Used for sending simulated Planning content to the control node.");

enum string_code {
  eChassis,
  eChassisDetail,
  eControlCommand,
  ePad,
  eDriveEvent,
  eGuardianCommand,
  eAudioCapture,
  eHMIStatus,
  eLocalization,
  eLocalizationStatus,
  eMonitorMessage,
  eSystemStatus,
  eNavigationInfo,
  ePerceptionObstacles,
  eTrafficLightDetection,
  eTrajectory,
  ePredictionObstacles,
  eRoutingRequest,
  eRoutingResponse,
  eGnssBestPose,
  eCorrectedImu,
  eGnssStatus,
  eImu,
  eInsStat,
  eGps,
  eRawData,
  eGnssEphemeris,
  eEpochObservation,
  eTransformStampeds,
  ePointCloud,
  eCompressedImage,
  eDefault
};

//      TEST FILE DEF END

const uint32_t PlayTaskProducer::kMinTaskBufferSize = 500;
const uint32_t PlayTaskProducer::kPreloadTimeSec = 3;
const uint64_t PlayTaskProducer::kSleepIntervalNanoSec = 1000000;

const std::map<std::string, std::string> msgTypeMap =
{
    { "/apollo/canbus/chassis", "apollo.canbus.Chassis"},
    { "/apollo/canbus/chassis_detail", "apollo.canbus.ChassisDetail" },
    { "/apollo/control", "apollo.control.ControlCommand" },
    { "/apollo/control/pad", "apollo.control.PadMessage" },
    { "/apollo/drive_event", "apollo.common.DriveEvent" },
    { "/apollo/guardian", "apollo.guardian.GuardianCommand" },
    { "/apollo/hmi/audio_capture", "apollo.dreamview.AudioCapture" },
    { "/apollo/hmi/status", "apollo.dreamview.HMIStatus" },
    { "/apollo/localization/msf_gnss", "apollo.localization.LocalizationEstimate" },
    { "/apollo/localization/msf_lidar", "apollo.localization.LocalizationEstimate" },
    { "/apollo/localization/msf_status", "apollo.localization.LocalizationStatus" },
    { "/apollo/localization/pose", "apollo.localization.LocalizationEstimate" },
    { "/apollo/monitor", "apollo.common.monitor.MonitorMessage" },
    { "/apollo/monitor/system_status", "apollo.monitor.SystemStatus" },
    { "/apollo/navigation", "apollo.relative_map.NavigationInfo" },
    { "/apollo/perception/obstacles", "apollo.perception.PerceptionObstacles" },
    { "/apollo/perception/traffic_light", "apollo.perception.TrafficLightDetection" },
    { "/apollo/planning", "apollo.planning.ADCTrajectory" },
    { "/apollo/prediction", "apollo.prediction.PredictionObstacles" },
    { "/apollo/routing_request", "apollo.routing.RoutingRequest" },
    { "/apollo/routing_response", "apollo.routing.RoutingResponse" },
    { "/apollo/sensor/gnss/best_pose", "apollo.drivers.gnss.GnssBestPose" },
    { "/apollo/sensor/gnss/corrected_imu", "apollo.localization.CorrectedImu" },
    { "/apollo/sensor/gnss/gnss_status", "apollo.drivers.gnss.GnssStatus" },
    { "/apollo/sensor/gnss/imu", "apollo.drivers.gnss.Imu" },
    { "/apollo/sensor/gnss/ins_stat", "apollo.drivers.gnss.InsStat" },
    { "/apollo/sensor/gnss/odometry", "apollo.localization.Gps" },
    { "/apollo/sensor/gnss/raw_data", "apollo.drivers.gnss.RawData" },
    { "/apollo/sensor/gnss/rtk_eph", "apollo.drivers.gnss.GnssEphemeris" },
    { "/apollo/sensor/gnss/rtk_obs", "apollo.drivers.gnss.EpochObservation" },
    { "/tf", "apollo.transform.TransformStampeds" },
    { "/tf_static", "apollo.transform.TransformStampeds" },
    { "/apollo/sensor/lidar128/compensator/PointCloud2", "apollo.drivers.PointCloud" },
    { "/apollo/sensor/camera/front_6mm/image/compressed", "apollo::drivers::CompressedImage" },
};

const std::vector<std::string> key, value;

//      TEST HASHIT START

string_code hashit (std::string const& inString) {
    if (inString == "/apollo/canbus/chassis") return eChassis;
    if (inString == "/apollo/canbus/chassis_detail") return eChassisDetail;
    if (inString == "/apollo/control") return eControlCommand;
    if (inString == "/apollo/control/pad") return ePad;
    if (inString == "/apollo/drive_event") return eDriveEvent;
    if (inString == "/apollo/guardian") return eGuardianCommand;
    if (inString == "/apollo/hmi/audio_capture") return eAudioCapture;
    if (inString == "/apollo/hmi/status") return eHMIStatus;
    if (inString == "/apollo/localization/msf_gnss") return eLocalization;
    if (inString == "/apollo/localization/msf_lidar") return eLocalization;
    if (inString == "/apollo/localization/pose") return eLocalization;
    if (inString == "/apollo/localization/msf_status") return eLocalizationStatus;
    if (inString == "/apollo/monitor") return eMonitorMessage;
    if (inString == "/apollo/monitor/system_status") return eSystemStatus;
    if (inString == "/apollo/navigation") return eNavigationInfo;
    if (inString == "/apollo/perception/obstacles") return ePerceptionObstacles;
    if (inString == "/apollo/perception/traffic_light") return eTrafficLightDetection;
    if (inString == "/apollo/planning") return eTrajectory;
    if (inString == "/apollo/prediction") return ePredictionObstacles;
    if (inString == "/apollo/routing_request") return eRoutingRequest;
    if (inString == "/apollo/routing_response") return eRoutingResponse;
    if (inString == "/apollo/sensor/gnss/best_pose") return eGnssBestPose;
    if (inString == "/apollo/sensor/gnss/corrected_imu") return eCorrectedImu;
    if (inString == "/apollo/sensor/gnss/gnss_status") return eGnssStatus;
    if (inString == "/apollo/sensor/gnss/imu") return eImu;
    if (inString == "/apollo/sensor/gnss/ins_stat") return eInsStat;
    if (inString == "/apollo/sensor/gnss/odometry") return eGps;
    if (inString == "/apollo/sensor/gnss/raw_data") return eRawData;
    if (inString == "/apollo/sensor/gnss/rtk_eph") return eGnssEphemeris;
    if (inString == "/apollo/sensor/gnss/rtk_obs") return eEpochObservation;
    if (inString == "/tf") return eTransformStampeds;
    if (inString == "/tf_static") return eTransformStampeds;
    if (inString == "/apollo/sensor/lidar128/compensator/PointCloud2") return ePointCloud;
    if (inString == "/apollo/sensor/camera/front_6mm/image/compressed") return eCompressedImage;
    return eDefault;
}

void testMessageContents()
{

    auto l = std::make_shared<apollo::localization::LocalizationEstimate>();

    const google::protobuf::Descriptor *descTest       = l->GetDescriptor();
    int fieldCountTest = descTest->field_count();

    for(int i = 0 ; i < fieldCountTest ; i++)
    {
      const google::protobuf::FieldDescriptor *field = descTest->field(i);

      if (field->message_type() != 0)
      {

        const google::protobuf::Descriptor *messagedDesc = field->message_type();
        int messageCount = messagedDesc->field_count();

        for(int j = 0 ; j < messageCount ; j++ )
        {
          const google::protobuf::FieldDescriptor *messageField = messagedDesc->field(j);

          std::cout << "\n " << messageField->name().c_str() << std::endl;

        }
      }
      else{

        std::cout << "\n " << field->name().c_str() << std::endl;

      }
    }
}

std::array<std::string, 6> assignMessageContents(std::vector<std::string> v)
{

//     auto l = std::make_shared<apollo::localization::LocalizationEstimate>();
//
//     const google::protobuf::Descriptor *descTest       = l->GetDescriptor();
//     int fieldCountTest = descTest->field_count();
//
//     for(int i = 0 ; i < fieldCountTest ; i++)
//     {
//       const google::protobuf::FieldDescriptor *field = descTest->field(i);
//
//       if (field->message_type() != 0)
//       {
//
//         const google::protobuf::Descriptor *messagedDesc = field->message_type();
//         int messageCount = messagedDesc->field_count();
//
//         for(int j = 0 ; j < messageCount ; j++ )
//         {
//           const google::protobuf::FieldDescriptor *messageField = messagedDesc->field(j);
//
//           std::cout << "\n " << messageField->name().c_str() << std::endl;
//
//         }
//       }
//       else{
//
//         std::cout << "\n " << field->name().c_str() << std::endl;
//
//       }
//     }

//       fprintf(stderr, "The name of the %i th element is %s and the type is  %s \n",i,field->name().c_str(),field->type_name());
//       std::cout << "\nThe name of the " << i << " th element is " << field->name().c_str() << " and the type is  " << field->type_name() << " and the extension count is  " << field->message_type() << std::endl;

//       else {
//
//         GetRepeatedString(const Message &message, const FieldDescriptor *field, int index)
//
//       }

  std::array<std::string, 6> modules;

  const google::protobuf::Descriptor *desc;

//   const google::protobuf::Descriptor *descState;

  int exceptionValue = 0;
  int additionalValue = 0;
  int repeatedValue = 1;
  int repeatedValue_i = 1;
  int e_Count = 0;
  int modulesCount = 0;
  int modulesState_i = 0;
  int modulesState_j = 0;
  int modulesState_k = 0;
  int modulesState_l = 0;

  int r_Count = 0;
//   bool hasContent;

  const std::string exception_fields[45]= { "module_name", "sequence_num", "lidar_timestamp", "camera_timestamp", "radar_timestamp", "version", "status", "chassis_error_mask", "chassis_gps", "engage_advice", "wheel_speed", "surround", "vehicle_id", "linear_acceleration", "angular_velocity", "uncertainty", "trajectory_point", "msf_status", "sensor_status", "timestamp", "is_dense", "intensity", "frame_type", "polygon_point", "tracking_time", "point_cloud", "confidence", "confidence_type", "drops", "acceleration", "anchor_point", "bbox2d", "sub_type", "measurements", "height_above_ground", "position_covariance", "velocity_covariance", "acceleration_covariance", "light_status" ,"msg","source", "v2x_info", "error_code", "lane_marker", "cipv_info"};

  for(int idx = 0 ; idx < 6 ; idx++)
  {

    if(idx == 0){
      auto m = std::make_shared<apollo::canbus::Chassis>();
      desc = m->GetDescriptor();
//       const google::protobuf::Reflection *refl       = m->GetReflection();
    }

    else if (idx == 1){
      auto m = std::make_shared<apollo::localization::LocalizationEstimate>();
      desc = m->GetDescriptor();
//       const google::protobuf::Reflection *refl       = m->GetReflection();
    }

    else if (idx == 2){
      continue;
      auto m = std::make_shared<apollo::transform::TransformStampeds>();
      desc = m->GetDescriptor();
//       const google::protobuf::Reflection *refl       = m->GetReflection();
    }

    else if (idx == 3){
      continue;
      auto m = std::make_shared<apollo::drivers::PointCloud>();
      desc = m->GetDescriptor();
//       const google::protobuf::Reflection *refl       = m->GetReflection();
    }

    else if (idx == 4){
      continue;
      auto m = std::make_shared<apollo::drivers::CompressedImage>();
      desc = m->GetDescriptor();
//       const google::protobuf::Reflection *refl       = m->GetReflection();
    }

    else if (idx == 5){
      auto m = std::make_shared<apollo::perception::PerceptionObstacles>();
      desc = m->GetDescriptor();
//       const google::protobuf::Reflection *refl       = m->GetReflection();
    }

//     descState  = 0;
//     exceptionValue = 0;

//     hasContent = true;
/*
    auto m = std::make_shared<apollo::canbus::Chassis>();
    auto l = std::make_shared<apollo::localization::LocalizationEstimate>();

    const google::protobuf::Descriptor *desc       = m->GetDescriptor();
    const google::protobuf::Reflection *refl       = m->GetReflection();*/

    std::stringstream ss;
    std::string str;

    int fieldCount = desc->field_count();

    for(int i = 0 ; i < fieldCount ; i++)
    {
      const google::protobuf::FieldDescriptor *field = desc->field(i);
//       std::cout << "\n " << chassis_items[i] << std::endl;
//       std::cout << "\n " << field->name().c_str() << std::endl;

      if(std::find(std::begin(exception_fields), std::end(exception_fields), field->name().c_str()) != std::end(exception_fields))
      {
          exceptionValue += 1;
      }
      else if (field->message_type() != 0 )
      {

        if (field->is_repeated())
        {
          repeatedValue = std::stoi(v[i - exceptionValue + modulesCount + e_Count + additionalValue]);

//           std::cout << "\n repeatedValue: " << std::stoi(v[i - exceptionValue + modulesCount + e_Count + additionalValue]) << "" << field->name().c_str() << std::endl;

          if (repeatedValue != 0){

            r_Count += std::stoi(v[i - exceptionValue + modulesCount + e_Count + additionalValue]) - 1;

            additionalValue += 1;

          }

//
        }

        for(int r1 = 0 ; r1 < repeatedValue ; r1++ )
        {
  //         if (field->message_type() != descState){
          ss << field->name().c_str() << " {" << "\n";

  //         std::cout << "\n ERROR " <<  field->name().c_str() << std::endl;
  // //           descState = field->message_type();
  // //         }
  //         std::cout << "\n" << " the name is : " << field->name().c_str() << " and the type name is : " << field->is_repeated() << std::endl;



          const google::protobuf::Descriptor *messagedDesc = field->message_type();
          int messageCount = messagedDesc->field_count();


          for(int j = 0 ; j < messageCount ; j++ )
          {
            const google::protobuf::FieldDescriptor *messageField = messagedDesc->field(j);

  //           hasContent = true;

            if(std::find(std::begin(exception_fields), std::end(exception_fields), messageField->name().c_str()) != std::end(exception_fields))
            {
  //             continue;
              exceptionValue += 1;
            }
            else if (messageField->message_type() != 0)
            {

              if (messageField->is_repeated())
              {

                repeatedValue_i = std::stoi(v[i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1]);

//                 std::cout << "\n repeatedValue: " << std::stoi(v[i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1]) << "" << messageField->name().c_str() << std::endl;


                if (repeatedValue_i != 0){

                  r_Count += std::stoi(v[i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1]) - 1;

//                   std::cout << "\n" << "r_Count: " << r_Count << std::endl;

                  additionalValue += 1;

                }

              }

              for(int r2 = 0 ; r2 < repeatedValue_i ; r2++ )
              {

                    //         if (field->message_type() != descState){
                ss << " " << messageField->name().c_str() << " {" << "\n";
        //           descState = field->message_type();
        //         }

                const google::protobuf::Descriptor *i_messagedDesc = messageField->message_type();
                int i_messageCount = i_messagedDesc->field_count();

    //             exceptionValue += 1;

                for(int k = 0 ; k < i_messageCount ; k++ )
                {
                  const google::protobuf::FieldDescriptor *i_messageField = i_messagedDesc->field(k);

        //           hasContent = true;



                  if(std::find(std::begin(exception_fields), std::end(exception_fields), i_messageField->name().c_str()) != std::end(exception_fields))
                  {
    //                 continue;
                    exceptionValue += 1;

                  }
                  else if (i_messageField->message_type() != 0 )
                  {

                    ss << "  " << i_messageField->name().c_str() << " {" << "\n";

                    const google::protobuf::Descriptor *j_messagedDesc = i_messageField->message_type();
                    int j_messageCount = j_messagedDesc->field_count();

                    for(int l = 0 ; l < j_messageCount ; l++ )
                    {
                      const google::protobuf::FieldDescriptor *j_messageField = j_messagedDesc->field(l);

                      if(std::find(std::begin(exception_fields), std::end(exception_fields), j_messageField->name().c_str()) != std::end(exception_fields))
                      {
        //                 continue;
                        exceptionValue += 1;
                      }
                      else{

                        ss << "   " << j_messageField->name().c_str() << ": " << v[i + j + k + l - exceptionValue + modulesCount + e_Count + additionalValue + r1 + r2] << "\n";

//                         std::cout << "\n" << j_messageField->name().c_str() << ": " << v[i + j + k + l - exceptionValue + modulesCount + e_Count + additionalValue + r1 + r2]<< std::endl;

                      }
                      modulesState_l = l;
                      }
                    e_Count += modulesState_l;
                    ss << "  }" << "\n";
                  }
                  else{


                    ss << "   " << i_messageField->name().c_str() << ": " << v[i + j + k - exceptionValue + modulesCount + e_Count + additionalValue + r1 + r2] << "\n";

//                     std::cout << "\n" << i_messageField->name().c_str() << ": " << v[i + j + k - exceptionValue + modulesCount + e_Count + additionalValue + r1 + r2] << " idx :" << e_Count <<  std::endl;


//                     if ((i + j + k - exceptionValue + modulesCount + e_Count + additionalValue + r1 + r2) < 50)
//                     {
//                       std::cout << "\n" << i_messageField->name().c_str() << ": " <<   (i + j + k - exceptionValue + modulesCount + e_Count + additionalValue + r1 + r2) << std::endl;
//                     }

    //                 std::cout << "\n" << " exceptionValue: " << exceptionValue << std::endl;
    //
    //                 std::cout << "\n" << " additionalValue: " << additionalValue << std::endl;
//                     std::cout << "\n" << " the name is : " << i_messageField->name().c_str() << " and the type name is : " << i_messageField->type_name() << std::endl;

                  }

                  modulesState_k = k;
                }
                e_Count += modulesState_k;
//                 additionalValue += 1;

        //         if (!hasContent)
        //         {
                ss << " }" << "\n";
        //         }
              }
              if (repeatedValue_i > 1){
                e_Count += r_Count;
              }
              repeatedValue_i = 1;
            }
            else{


              ss << " " << messageField->name().c_str() << ": " << v[i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1] << "\n";

//               std::cout << "\n" << messageField->name().c_str() << ": " << v[i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1] << " idx :" << e_Count << std::endl;


//               if ((i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1) < 50)
//               {
//                 std::cout << "\n" << messageField->name().c_str() << " : " << (i + j - exceptionValue + modulesCount + e_Count + additionalValue + r1) << std::endl;
//               }
  //             std::cout << "\n" << " exceptionValue: " << exceptionValue << std::endl;
  //
  //             std::cout << "\n" << " additionalValue: " << additionalValue << std::endl;
//               std::cout << "\n" << " the name is : " << messageField->name().c_str() << " and the type name is : " << messageField->type_name() << std::endl;

            }

            modulesState_j = j;
          }

          e_Count += modulesState_j;
//           additionalValue += 1;

  //         if (!hasContent)
  //         {
          ss << "}" << "\n";
  //         }
        }
        if (repeatedValue > 1){
          e_Count += r_Count;
//           std::cout << "\n" << "new e_Count: " << e_Count << std::endl;

        }
        r_Count = 0;
        repeatedValue = 1;
      }
      else{

//         if (field->name() == "width" || field->name() == "height")
//         {
//           ss << field->name().c_str() << ": " << i - exceptionValue + modulesCount + e_Count + additionalValue + r_test << " | " << static_cast<int>(v.size()) << "\n";
//         }
//         else{
        ss << field->name().c_str() << ": " << v[i - exceptionValue + modulesCount + e_Count + additionalValue] << "\n";

//         std::cout << "\n" << field->name().c_str() << ": " << v[i - exceptionValue + modulesCount + e_Count + additionalValue] << " idx :" << e_Count << std::endl;

//         }



//         if ((i - exceptionValue + modulesCount + e_Count + additionalValue) < 50)
//         {
//           std::cout << "\n" << field->name().c_str() << ": " << (i - exceptionValue + modulesCount + e_Count + additionalValue) << std::endl;
//         }
//         std::cout << "\n" << " exceptionValue: " << exceptionValue << std::endl;
//
//         std::cout << "\n" << " additionalValue: " << additionalValue << std::endl;
//         std::cout << "\n" << " the name is : " << field->name().c_str() << " and the type name is : " << field->type_name() << std::endl;
      }


//       fprintf(stderr, "The name of the %i th element is %s and the type is  %s \n",i,field->name().c_str(),field->type_name());
//       std::cout << "\nThe name of the " << i << " th element is " << field->name().c_str() << " and the type is  " << field->type_name() << " and the extension count is  " << field->message_type() << std::endl;


//       if(field->type() == google::protobuf::FieldDescriptor::TYPE_STRING  && !field->is_repeated())
//       {
//         std::string g= refl->GetString(*m, field);
//         fprintf(stderr, "The value is %s ",g.c_str());
//       }

//       else {
//
//         GetRepeatedString(const Message &message, const FieldDescriptor *field, int index)
//
//       }

      modulesState_i = i;
    }
    e_Count += modulesState_i;
    additionalValue += 1;
//     std::cout << "\n e_Count: " << e_Count << "\n exception_value: " << exceptionValue << "\n additional_value: " << additionalValue <<  std::endl;
// //     e_Count = 0;
// //     modulesCount += (modulesState_i + modulesState_j + modulesState_k);
// //     exceptionValue = 0;
//     std::cout << "\n modulesCount: " << exceptionValue << std::endl;
    str = ss.str();
//     std::cout << "\n" << str << std::endl;
    modules[idx] = str;

  }
//   std::cout << "\n" << modules[0] << std::endl;
//   std::cout << "\n" << modules[1] << std::endl;
  return modules;

}

//      TEST HASHIT END

PlayTaskProducer::PlayTaskProducer(const TaskBufferPtr& task_buffer,
                                   const PlayParam& play_param)
    : play_param_(play_param),
      task_buffer_(task_buffer),
      produce_th_(nullptr),
      is_initialized_(false),
      is_stopped_(true),
      node_(nullptr),
      earliest_begin_time_(std::numeric_limits<uint64_t>::max()),
      latest_end_time_(0),
      total_msg_num_(0) {}

PlayTaskProducer::~PlayTaskProducer() { Stop(); }

bool PlayTaskProducer::Init() {
  if (is_initialized_.exchange(true)) {
    AERROR << "producer has been initialized.";
    return false;
  }

  if (!ReadRecordInfo() || !CreateWriters()) {
    is_initialized_.store(false);
    return false;
  }

  return true;
}

void PlayTaskProducer::Start(std::string message) {

  receiverMsg_ = message;

  if (!is_initialized_.load()) {
    AERROR << "please call Init firstly.";
    return;
  }

  if (!is_stopped_.exchange(false)) {
    AERROR << "producer has been started.";
    return;
  }

  produce_th_.reset(new std::thread(&PlayTaskProducer::ThreadFunc, this));
}

void PlayTaskProducer::Stop() {
  if (!is_stopped_.exchange(true)) {
    return;
  }
  if (produce_th_ != nullptr && produce_th_->joinable()) {
    produce_th_->join();
    produce_th_ = nullptr;
  }
}

bool PlayTaskProducer::ReadRecordInfo() {

  std::set<std::string> channel_list;

  for(auto it = msgTypeMap.begin(); it != msgTypeMap.end(); ++it) {
    channel_list.insert(it->first);
  }

  // loop each channel info
  for (auto& channel_name : channel_list) {

//     if (play_param_.black_channels.find(channel_name) !=
//         play_param_.black_channels.end()) {
      // minus the black message number from record file header
//       total_msg_num_ -= record_reader->GetMessageNumber(channel_name);
//       continue;
//     }

    auto& msg_type = msgTypeMap.find(channel_name)->second;
    msg_types_[channel_name] = msg_type;

//     if (!play_param_.is_play_all_channels &&
//         play_param_.channels_to_play.count(channel_name) > 0) {
//       total_msg_num_ += record_reader->GetMessageNumber(channel_name);
//     }
  }

  return true;
}

bool PlayTaskProducer::UpdatePlayParam() {
  if (play_param_.begin_time_ns < earliest_begin_time_) {
    play_param_.begin_time_ns = earliest_begin_time_;
  }
  if (play_param_.start_time_s > 0) {
    play_param_.begin_time_ns += static_cast<uint64_t>(
        static_cast<double>(play_param_.start_time_s) * 1e9);
  }
  if (play_param_.end_time_ns > latest_end_time_) {
    play_param_.end_time_ns = latest_end_time_;
  }
  if (play_param_.begin_time_ns >= play_param_.end_time_ns) {
    AERROR << "begin time are equal or larger than end time"
           << ", begin_time_ns=" << play_param_.begin_time_ns
           << ", end_time_ns=" << play_param_.end_time_ns;
    return false;
  }
  if (play_param_.preload_time_s == 0) {
    AINFO << "preload time is zero, we will use defalut value: "
          << kPreloadTimeSec << " seconds.";
    play_param_.preload_time_s = kPreloadTimeSec;
  }
  return true;
}


bool PlayTaskProducer::CreateWriters() {
  std::string node_name = "cyber_recorder_play_" + std::to_string(getpid());
  node_ = apollo::cyber::CreateNode(node_name);
  if (node_ == nullptr) {
    AERROR << "create node failed.";
    return false;
  }

  for (auto& item : msg_types_) {
    auto& channel_name = item.first;
    auto& msg_type = item.second;

    if (play_param_.is_play_all_channels ||
        play_param_.channels_to_play.count(channel_name) > 0) {
      if (play_param_.black_channels.find(channel_name) !=
          play_param_.black_channels.end()) {
        continue;
      }

//   for (int i = 0; i < 50; i++) {



      switch(hashit(channel_name)) {

          case eChassis:
            {
            auto writer = node_->CreateWriter<Chassis>(channel_name);

            if (writer == nullptr) {
              AERROR << "create writer failed. channel name: " << channel_name
                      << ", message type: " << msg_type;
              return false;
            }
//             writers_[channel_name] = writer;
            chassis_writer_ = writer;

            break;
            }

//           case ePad:
//             {
//             auto writer = node_->CreateWriter<PadMessage>(channel_name);
//
//             if (writer == nullptr) {
//               AERROR << "create writer failed. channel name: " << channel_name
//                       << ", message type: " << msg_type;
//               return false;
//             }
// //             writers_[channel_name] = writer;
//             pad_msg_writer_ = writer;
//
//             break;
//             }

          case eLocalization:
            {
            auto writer = node_->CreateWriter<LocalizationEstimate>(channel_name);

            if (writer == nullptr) {
              AERROR << "create writer failed. channel name: " << channel_name
                      << ", message type: " << msg_type;
              return false;
            }
//             writers_[channel_name] = writer;
            localization_writer_ = writer;

            break;
            }

//           case eTransformStampeds:
//             {
//             auto writer = node_->CreateWriter<TransformStampeds>(channel_name);
//
//             if (writer == nullptr) {
//               AERROR << "create writer failed. channel name: " << channel_name
//                       << ", message type: " << msg_type;
//               return false;
//             }
// //             writers_[channel_name] = writer;
//             tf_writer_ = writer;
//
//             break;
//             }

//           case ePointCloud:
//             {
//             auto writer = node_->CreateWriter<PointCloud>(channel_name);
//
//             if (writer == nullptr) {
//               AERROR << "create writer failed. channel name: " << channel_name
//                       << ", message type: " << msg_type;
//               return false;
//             }
// //             writers_[channel_name] = writer;
//             pointcloud_writer_ = writer;
//
//             break;
//             }
//
//           case eCompressedImage:
//             {
//             auto writer = node_->CreateWriter<CompressedImage>(channel_name);
//
//             if (writer == nullptr) {
//               AERROR << "create writer failed. channel name: " << channel_name
//                       << ", message type: " << msg_type;
//               return false;
//             }
// //             writers_[channel_name] = writer;
//             compressedimage_writer_ = writer;
//
//             break;
//             }

          case ePerceptionObstacles:
            {
            auto writer = node_->CreateWriter<PerceptionObstacles>(channel_name);

            if (writer == nullptr) {
              AERROR << "create writer failed. channel name: " << channel_name
                      << ", message type: " << msg_type;
              return false;
            }
//             writers_[channel_name] = writer;
            perceptionobstacles_writer_ = writer;

            break;
            }

//           case eTrajectory:
//             {
//             auto writer = node_->CreateWriter<ADCTrajectory>(channel_name);
//
//             if (writer == nullptr) {
//               AERROR << "create writer failed. channel name: " << channel_name
//                       << ", message type: " << msg_type;
//               return false;
//             }
// //             writers_[channel_name] = writer;
//             trajectory_writer_ = writer;
//
//             break;
//             }
          default :
//             proto::RoleAttributes attr;
//             attr.set_channel_name(channel_name);
//             attr.set_message_type(msg_type);
//             auto writer = node_->CreateWriter<message::RawMessage>(attr);

            break;
      }




//       proto::RoleAttributes attr;
//       attr.set_channel_name(channel_name);
//       attr.set_message_type(msg_type);
//       auto writer = node_->CreateWriter<message::RawMessage>(attr);

//       if (writer == nullptr) {
//         AERROR << "create writer failed. channel name: " << channel_name
//                << ", message type: " << msg_type;
//         return false;
//       }
//       writers_[channel_name] = writer;
    }
  }

  return true;
}

void PlayTaskProducer::ThreadFunc() {

//    testMessageContents();

//   std::cout << "\n" << receiverMessage << "\n" << std::endl;
  std::stringstream ss(receiverMsg_);
  std::vector<std::string> receiverVect;
//   const char s = '´';

  while(ss.good())
  {
    std::string substr;
    getline(ss, substr, '\n');
    receiverVect.push_back(substr);
  }

//   std::cout << "\n" << receiverVect << "\n" << std::endl;

//   for (unsigned i = 0; i < receiverVect.size(); i++) {
//     std::cout << receiverVect[i] << " " << std::endl;
//   }


//   std::shared_ptr<Chassis> chassis_test;
//   std::string str;
//   std::string str[2];


  std::array<std::string, 6> str = assignMessageContents(receiverVect);

//   std::cout << "\n" << str[5] << "\n" << std::endl;

  Chassis chassis_content;
  if (!GetProtoFromString(str[0], &chassis_content)) {
    AERROR << "failed to fill chassis_content";
  }
  auto chassis = std::make_shared<apollo::canbus::Chassis>(chassis_content);

  LocalizationEstimate localization_content;
  if (!GetProtoFromString(str[1], &localization_content)) {
    AERROR << "failed to fill localization_content";
  }
  auto localization = std::make_shared<apollo::localization::LocalizationEstimate>(localization_content);

//   TransformStampeds tf_content;
//   if (!GetProtoFromString(str[2], &tf_content)) {
//     AERROR << "failed to fill localization_content";
//   }
//   auto tf = std::make_shared<apollo::transform::TransformStampeds>(tf_content);

//   PointCloud pointcloud_content;
//   if (!GetProtoFromString(str[3], &pointcloud_content)) {
//     AERROR << "failed to fill pointcloud_content";
//   }
//   auto point_cloud = std::make_shared<apollo::drivers::PointCloud>(pointcloud_content);
//
//   CompressedImage compressedimage_content;
//   if (!GetProtoFromString(str[4], &compressedimage_content)) {
//     AERROR << "failed to fill pointcloud_content";
//   }
//   auto compressed_image = std::make_shared<apollo::drivers::CompressedImage>(compressedimage_content);

  PerceptionObstacles perceptionobstacles_content;
  if (!GetProtoFromString(str[5], &perceptionobstacles_content)) {
    AERROR << "failed to fill pointcloud_content";
  }
  auto perception_obstacles = std::make_shared<apollo::perception::PerceptionObstacles>(perceptionobstacles_content);

//   PadMessage pad_msg_content;
//   if (!GetProtoFromFile(FLAGS_pad_msg_test_file, &pad_msg_content)) {
//     AERROR << "failed to load file: " << FLAGS_pad_msg_test_file;
//   }
//   auto pad_msg = std::make_shared<apollo::control::PadMessage>(pad_msg_content);
//
//
//
//   ADCTrajectory trajectory_content;
//   if (!GetProtoFromFile(FLAGS_planning_test_file, &trajectory_content)) {
//     AERROR << "failed to load file: " << FLAGS_planning_test_file;
//   }
//   auto trajectory = std::make_shared<apollo::planning::ADCTrajectory>(trajectory_content);

//      TEST GET PROTO END

  const uint64_t loop_time_ns =
      play_param_.end_time_ns - play_param_.begin_time_ns;
  uint64_t avg_interval_time_ns = kSleepIntervalNanoSec;
  if (total_msg_num_ > 0) {
    avg_interval_time_ns = loop_time_ns / total_msg_num_;
  }

  double avg_freq_hz = static_cast<double>(total_msg_num_) /
                       (static_cast<double>(loop_time_ns) * 1e-9);
  uint32_t preload_size = (uint32_t)avg_freq_hz * play_param_.preload_time_s;
  AINFO << "preload_size: " << preload_size;
  if (preload_size < kMinTaskBufferSize) {
    preload_size = kMinTaskBufferSize;
  }

//   auto record_viewer = std::make_shared<RecordViewer>(
//       record_readers_, play_param_.begin_time_ns, play_param_.end_time_ns,
//       play_param_.channels_to_play);

//   uint32_t loop_num = 0;
  while (!is_stopped_.load()) {
//     uint64_t plus_time_ns = loop_num * loop_time_ns;
//     auto itr = record_viewer->begin();
//     auto itr_end = record_viewer->end();

//     while (itr != itr_end && !is_stopped_.load()) {
      while (!is_stopped_.load() && task_buffer_->Size() > preload_size) {
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(avg_interval_time_ns));
      }
//       for (; itr != itr_end && !is_stopped_.load(); ++itr) {

      for (auto& item : msg_types_) {
        auto& channel_name = item.first;

        if (task_buffer_->Size() > preload_size) {
          break;
        }

//         auto search = writers_.find(itr->channel_name);
//         if (search == writers_.end()) {
//           continue;
//         }

//         auto raw_msg = std::make_shared<message::RawMessage>(itr->content);

//         std::cout << "\ritr_content input type : " << typeid(itr->content).name() << std::endl;

//         std::cout << "\ritr_content for " << itr->channel_name << " : " << itr->content << std::endl;
//         std::cout << "\ritr_raw_msg for " << itr->channel_name << " : " << raw_msg << std::endl;

//      TEST TASK START

        switch(hashit(/*itr->*/channel_name)) {
          case eChassis:
              {
              auto task = std::make_shared<PlayTask>(
                chassis, chassis_writer_, Time::Now().ToNanosecond());
              task_buffer_->Push(task);
//               std::cout << "\rchassis\r" << std::endl;

              break;
              }
//           case ePad:
//               {
//               auto task = std::make_shared<PlayTask>(
//                 pad_msg, pad_msg_writer_, Time::Now().ToNanosecond());
//               task_buffer_->Push(task);
// //               std::cout << "\rpad\r" << std::endl;
//
//               break;
//               }
          case eLocalization:
              {
              auto task = std::make_shared<PlayTask>(
                localization, localization_writer_, Time::Now().ToNanosecond());
              task_buffer_->Push(task);
  //               std::cout << "\rlocal\r" << std::endl;

              break;
              }

//           case eTransformStampeds:
//             {
//             auto task = std::make_shared<PlayTask>(
//               tf, tf_writer_, Time::Now().ToNanosecond());
//             task_buffer_->Push(task);
// //               std::cout << "\rlocal\r" << std::endl;
//
//             break;
//             }
//           case eTrajectory:
//               {
//               auto task = std::make_shared<PlayTask>(
//                 trajectory, trajectory_writer_, Time::Now().ToNanosecond());
//               task_buffer_->Push(task);
// //               std::cout << "\rtraj\r" << std::endl;
//
//               break;
//               }

//           case ePointCloud:
//               {
//               auto task = std::make_shared<PlayTask>(
//                 point_cloud, pointcloud_writer_, Time::Now().ToNanosecond());
//               task_buffer_->Push(task);
//   //               std::cout << "\rlocal\r" << std::endl;
//
//               break;
//               }
//
//           case eCompressedImage:
//               {
//               auto task = std::make_shared<PlayTask>(
//                 compressed_image, compressedimage_writer_, Time::Now().ToNanosecond());
//               task_buffer_->Push(task);
//   //               std::cout << "\rlocal\r" << std::endl;
//
//               break;
//               }

          case ePerceptionObstacles:
              {
              auto task = std::make_shared<PlayTask>(
                perception_obstacles, perceptionobstacles_writer_, Time::Now().ToNanosecond());
              task_buffer_->Push(task);
  //               std::cout << "\rlocal\r" << std::endl;

              break;
              }

          case eDefault:
//             auto task = std::make_shared<PlayTask>(
//             trajectory, search->second, itr->time, itr->time + plus_time_ns);
//             task_buffer_->Push(task);
//             auto task = std::make_shared<PlayTask>(
//               raw_msg, search->second, itr->time, itr->time + plus_time_ns);
//             task_buffer_->Push(task);
//
            break;

          default:

            break;
        }

//      TEST TASK END

//       std::cout << "\rtime 0 for " << itr->channel_name << " : " << itr->time << std::endl;

//       auto task = std::make_shared<PlayTask>(
//         raw_msg, search->second, itr->time, itr->time + plus_time_ns);
//       task_buffer_->Push(task);

//       std::cout << "\rtime F for " << itr->channel_name << " : " << (loop_num * loop_time_ns) << std::endl;

      }
//     }

    if (!play_param_.is_loop_playback) {
      is_stopped_.store(true);
      break;
    }
//     ++loop_num;
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
