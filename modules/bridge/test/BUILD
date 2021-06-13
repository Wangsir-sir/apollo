load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "bridge_receiver_test",
    srcs = ["bridge_receiver_test.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:udp_bridge",
        "//modules/bridge/common:udp_listener",
    ],
)

cc_binary(
    name = "bridge_sender_test_Chassis",
    srcs = ["bridge_sender_test_Chassis.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:udp_bridge",
    ],
)

cc_binary(
    name = "bridge_sender_test_PointCloud",
    srcs = ["bridge_sender_test_PointCloud.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:udp_bridge",
        "//modules/drivers/proto:pointcloud_cc_proto",

    ],
)

cpplint()
