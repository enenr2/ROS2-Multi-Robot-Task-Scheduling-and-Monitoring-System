// cmd_vel 安全链路：停车帧数、非法输入、QoS 语义的纯逻辑单测。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "fleet_agent/cmd_vel_link.hpp"

namespace {

using Command = std::pair<double, double>;

struct Recorder {
  std::vector<Command> commands;

  fleet::CmdVelLink Make(int frames = 3)
  {
    return fleet::CmdVelLink(
      [this](double linear, double angular) { commands.emplace_back(linear, angular); },
      fleet::StopPolicy{frames});
  }
};

}  // namespace

TEST(CmdVelLink, StopPublishesConfiguredFrames)
{
  Recorder recorder;
  auto link = recorder.Make(3);
  link.Stop();

  ASSERT_EQ(recorder.commands.size(), 3u);
  for (const auto& [linear, angular] : recorder.commands) {
    EXPECT_DOUBLE_EQ(linear, 0.0);
    EXPECT_DOUBLE_EQ(angular, 0.0);
  }
}

TEST(CmdVelLink, FrameCountIsAtLeastOne)
{
  Recorder recorder;
  auto link = recorder.Make(0);
  link.Stop();
  EXPECT_EQ(recorder.commands.size(), 1u) << "0 帧停车等于没停";
}

TEST(CmdVelLink, DrivePublishesOnce)
{
  Recorder recorder;
  auto link = recorder.Make();
  EXPECT_TRUE(link.Drive(0.12, -0.3));
  ASSERT_EQ(recorder.commands.size(), 1u);
  EXPECT_DOUBLE_EQ(recorder.commands[0].first, 0.12);
  EXPECT_DOUBLE_EQ(recorder.commands[0].second, -0.3);
}

TEST(CmdVelLink, InvalidInputStopsInsteadOfPublishing)
{
  Recorder recorder;
  auto link = recorder.Make(2);

  EXPECT_FALSE(link.Drive(std::nan(""), 0.1));
  ASSERT_EQ(recorder.commands.size(), 2u) << "NaN 必须转成停车帧";
  EXPECT_DOUBLE_EQ(recorder.commands[0].first, 0.0);
  EXPECT_DOUBLE_EQ(recorder.commands[1].first, 0.0);

  recorder.commands.clear();
  EXPECT_FALSE(link.Drive(0.1, std::numeric_limits<double>::infinity()));
  ASSERT_EQ(recorder.commands.size(), 2u);
}