load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "play_param",
    hdrs = ["play_param.h"],
)

cc_library(
    name = "play_task",
    srcs = ["play_task.cc"],
    hdrs = ["play_task.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/message:raw_message",
        "//cyber/node:writer",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util",
        "//modules/control/common:control_gflags",
        "//modules/control/proto:pad_msg_cc_proto",
        "//modules/dreamview/proto:chart_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "//modules/map/proto:map_cc_proto",
        "//modules/map/proto:map_clear_area_cc_proto",
        "//modules/map/proto:map_crosswalk_cc_proto",
        "//modules/map/proto:map_geometry_cc_proto",
        "//modules/map/proto:map_id_cc_proto",
        "//modules/map/proto:map_junction_cc_proto",
        "//modules/map/proto:map_lane_cc_proto",
        "//modules/map/proto:map_overlap_cc_proto",
        "//modules/map/proto:map_parking_space_cc_proto",
        "//modules/map/proto:map_pnc_junction_cc_proto",
        "//modules/map/proto:map_road_cc_proto",
        "//modules/map/proto:map_signal_cc_proto",
        "//modules/map/proto:map_speed_bump_cc_proto",
        "//modules/map/proto:map_stop_sign_cc_proto",
        "//modules/map/proto:map_yield_sign_cc_proto",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/planning/proto:decision_cc_proto",
        "//modules/planning/proto:open_space_task_config_cc_proto",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/proto:planning_internal_cc_proto",
        "//modules/planning/proto:sl_boundary_cc_proto",
        "//modules/planning/proto:task_config_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
    ],
)

cc_library(
    name = "play_task_buffer",
    srcs = ["play_task_buffer.cc"],
    hdrs = ["play_task_buffer.h"],
    deps = [
        ":play_task",
    ],
)

cc_library(
    name = "play_task_consumer",
    srcs = ["play_task_consumer.cc"],
    hdrs = ["play_task_consumer.h"],
    deps = [
        ":play_task_buffer",
        "//cyber/common:log",
        "//cyber/time",
    ],
)

include_files = [
    "zmq.hpp",
    "zmq.h",
]

lib_files = [
    "libzmq.a",
]

cc_library(
  name = "zmq",
  srcs = lib_files,
  hdrs = include_files,
  linkstatic = 1,
)

cc_library(
    name = "play_task_producer",
    srcs = ["play_task_producer.cc"],
    hdrs = ["play_task_producer.h"],
    deps = [
        ":play_param",
        ":play_task_buffer",
        ":zmq",
        "//cyber",
        "//cyber/common:log",
        "//cyber/message:protobuf_factory",
        "//cyber/message:raw_message",
        "//cyber/node",
        "//cyber/node:writer",
        "//cyber/record:record_reader",
        "//cyber/record:record_viewer",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "play_task_sender",
    srcs = ["play_task_sender.cc"],
    hdrs = ["play_task_sender.h"],
    deps = [
        ":zmq",
        "//cyber",
        "//cyber/message:raw_message",
    ],
)

cc_library(
    name = "player",
    srcs = ["player.cc"],
    hdrs = ["player.h"],
    deps = [
        ":play_param",
        ":play_task_buffer",
        ":play_task_consumer",
        ":play_task_producer",
        ":play_task_sender",
        ":zmq",
        "//cyber:init",
    ],
)

cpplint()
