// Copyright (c) 2026, Davide Faconti
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>
#include <tuple>
#include <vector>

#include <image_transport/publisher_plugin.hpp>
#include <image_transport/subscriber_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "depth_codec.hpp"

using namespace std::chrono_literals;

class DepthzTransport : public testing::TestWithParam<std::tuple<bool, bool, bool>>
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_P(DepthzTransport, lossless_plugin_roundtrip_and_replay)
{
  const auto [float_pixels, big_endian, padded] = GetParam();
  pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader(
    "image_transport", "image_transport::PublisherPlugin");
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");
  auto pub = pub_loader.createSharedInstance("image_transport/depthz_pub");
  auto sub = sub_loader.createSharedInstance("image_transport/depthz_sub");
  auto node = std::make_shared<rclcpp::Node>("depthz_roundtrip", "/prbatch");
  pub->advertise(node.get(), "/prbatch/depth");
  sensor_msgs::msg::Image::ConstSharedPtr received;
  sub->subscribe(node.get(), "/prbatch/depth",
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & image) {received = image;});
  sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
  auto raw_sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/prbatch/depth/depthz", 10,
    [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {compressed = image;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto wait = [&](auto condition) {
      const auto end = std::chrono::steady_clock::now() + 5s;
      while (!condition() && std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
      return condition();
    };
  ASSERT_TRUE(wait([&]() {return pub->getNumSubscribers() == 2;}));
  sensor_msgs::msg::Image input;
  input.width = 4;
  input.height = 3;
  input.header.frame_id = "depth_optical_frame";
  input.header.stamp.sec = 123;
  input.encoding = float_pixels ? "32FC1" : "16UC1";
  input.is_bigendian = big_endian;
  const size_t bpp = float_pixels ? 4 : 2;
  input.step = input.width * bpp + (padded ? 3 : 0);
  input.data.resize(input.step * input.height, 0xaa);
  const std::vector<uint32_t> patterns{
    0, 0x80000000, 0x7fc01234, 0x7f800000, 0xff800000, 0x3f800001,
    0x3f800000, 0x40000000, 0x7fa01234, 0xffffffff, 0x00010001, 0x0000ffff};
  std::vector<uint8_t> expected;
  for (size_t i = 0; i < patterns.size(); ++i) {
    for (size_t byte = 0; byte < bpp; ++byte) {
      expected.push_back(static_cast<uint8_t>(patterns[i] >> (byte * 8)));
      const auto position = (i / input.width) * input.step + (i % input.width) * bpp;
      input.data[position + byte] = static_cast<uint8_t>(
        patterns[i] >> ((big_endian ? bpp - 1 - byte : byte) * 8));
    }
  }
  pub->publish(input);
  ASSERT_TRUE(wait([&]() {return received && compressed;}));
  EXPECT_EQ(received->data, expected);
  EXPECT_EQ(received->encoding, input.encoding);
  EXPECT_EQ(received->step, input.width * bpp);
  EXPECT_EQ(received->header, input.header);
  EXPECT_EQ(compressed->format, input.encoding + "; depthz");
  EXPECT_EQ(depth_codec::read_header(compressed->data.data(), compressed->data.size()).
    quantization_step, 0.0f);

  auto replay = node->create_publisher<sensor_msgs::msg::CompressedImage>(
    "/prbatch/depth/depthz", 10);
  ASSERT_TRUE(wait([&]() {return replay->get_subscription_count() == 2;}));
  received.reset();
  pub->shutdown();
  replay->publish(*compressed);
  ASSERT_TRUE(wait([&]() {return received != nullptr;}));
  EXPECT_EQ(received->data, expected);
  EXPECT_EQ(received->header, input.header);
}

INSTANTIATE_TEST_SUITE_P(
  EncodingsAndLayouts, DepthzTransport,
  testing::Combine(testing::Bool(), testing::Bool(), testing::Bool()));
