load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_sender",
    srcs = [
        "cyber_topology_message.cc",
        "general_channel_message.cc",
        "general_message.cc",
        "general_message_base.cc",
        "main.cc",
        "renderable_message.cc",
        "screen.cc",
    ],
    linkopts = ["-pthread"],
    deps = [
        ":cyber_topology_message",
        ":general_channel_message",
        ":screen",
        "//cyber:init",
        "//cyber/service_discovery:topology_manager",
        "@ncurses5",
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
    name = "cyber_topology_message",
    hdrs = ["cyber_topology_message.h"],
    deps = [
        ":renderable_message",
    ],
)

cc_library(
    name = "general_channel_message",
    hdrs = ["general_channel_message.h"],
    deps = [
        ":general_message",
        ":general_message_base",
        ":screen",
        ":zmq",
        "//cyber",
        "//cyber/message:raw_message",
        "//cyber/record:record_message",
    ],
)

cc_library(
    name = "general_message",
    hdrs = ["general_message.h"],
    deps = [
        ":general_message_base",
        "//cyber",
        "//cyber/message:raw_message",
    ],
)

cc_library(
    name = "general_message_base",
    hdrs = ["general_message_base.h"],
    deps = [
        ":renderable_message",
    ],
)

cc_library(
    name = "renderable_message",
    hdrs = ["renderable_message.h"],
    deps = [
        ":screen",
    ],
)

cc_library(
    name = "screen",
    hdrs = ["screen.h"],
)

cpplint()
