#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "fleet_perception/color_detector.hpp"

using fleet::Blob;
using fleet::Color;
using fleet::ColorDetector;

namespace {

// 在 320x240 的深灰背景上画一个纯色矩形
cv::Mat MakeImage(const cv::Scalar& bgr, const cv::Rect& rect)
{
  cv::Mat img(240, 320, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::rectangle(img, rect, bgr, cv::FILLED);
  return img;
}

}  // namespace

TEST(ColorDetector, DetectsRedBlock)
{
  ColorDetector detector;
  const cv::Mat img = MakeImage(cv::Scalar(0, 0, 220), cv::Rect(100, 80, 40, 40));

  const auto blobs = detector.Detect(img);
  ASSERT_EQ(blobs.size(), 1u);
  EXPECT_EQ(blobs[0].color, Color::kRed);
  EXPECT_NEAR(blobs[0].bbox.CenterU(), 120.0, 1.0);
  EXPECT_NEAR(blobs[0].bbox.CenterV(), 100.0, 1.0);
  EXPECT_NEAR(blobs[0].bbox.width, 40, 1);
}

TEST(ColorDetector, DetectsGreenAndBlue)
{
  ColorDetector detector;

  const auto green = detector.Detect(MakeImage(cv::Scalar(0, 200, 0), cv::Rect(10, 10, 30, 30)));
  ASSERT_EQ(green.size(), 1u);
  EXPECT_EQ(green[0].color, Color::kGreen);

  const auto blue = detector.Detect(MakeImage(cv::Scalar(220, 0, 0), cv::Rect(10, 10, 30, 30)));
  ASSERT_EQ(blue.size(), 1u);
  EXPECT_EQ(blue[0].color, Color::kBlue);
}

TEST(ColorDetector, ReportsAtMostOneBlobPerColor)
{
  ColorDetector detector;
  cv::Mat img(240, 320, CV_8UC3, cv::Scalar(40, 40, 40));
  // 两个红块，一大一小，应只报面积大的那个
  cv::rectangle(img, cv::Rect(20, 20, 20, 20), cv::Scalar(0, 0, 220), cv::FILLED);
  cv::rectangle(img, cv::Rect(200, 150, 60, 60), cv::Scalar(0, 0, 220), cv::FILLED);

  const auto blobs = detector.Detect(img);
  ASSERT_EQ(blobs.size(), 1u);
  EXPECT_EQ(blobs[0].color, Color::kRed);
  EXPECT_NEAR(blobs[0].bbox.CenterU(), 230.0, 1.0);
  EXPECT_GT(blobs[0].area, 2500);
}

TEST(ColorDetector, IgnoresBackground)
{
  ColorDetector detector;
  const cv::Mat img(240, 320, CV_8UC3, cv::Scalar(40, 40, 40));
  EXPECT_TRUE(detector.Detect(img).empty());
}

TEST(ColorDetector, IgnoresGreyDespiteBright)
{
  ColorDetector detector;
  // 很亮的灰色（S=0），不能被误判成任何颜色
  const cv::Mat img = MakeImage(cv::Scalar(240, 240, 240), cv::Rect(50, 50, 80, 80));
  EXPECT_TRUE(detector.Detect(img).empty());
}

TEST(ColorDetector, IgnoresTinySpeck)
{
  ColorDetector detector;
  const cv::Mat img = MakeImage(cv::Scalar(0, 0, 220), cv::Rect(100, 100, 4, 4));
  EXPECT_TRUE(detector.Detect(img).empty());
}

TEST(ColorDetector, ConfidenceScalesWithArea)
{
  ColorDetector detector;
  const auto small = detector.Detect(MakeImage(cv::Scalar(0, 200, 0), cv::Rect(10, 10, 20, 20)));
  const auto big = detector.Detect(MakeImage(cv::Scalar(0, 200, 0), cv::Rect(10, 10, 60, 60)));
  ASSERT_EQ(small.size(), 1u);
  ASSERT_EQ(big.size(), 1u);
  EXPECT_GT(big[0].confidence, small[0].confidence);
  // 面积 9 倍，但 OpenCV 的 contourArea 对实心矩形按内边界计算，
  // 小目标相对误差更大，实测比值约 9.6
  EXPECT_NEAR(big[0].confidence / small[0].confidence, 9.0, 1.0);
}

TEST(ColorDetector, RejectsInvalidInput)
{
  ColorDetector detector;
  EXPECT_TRUE(detector.Detect(cv::Mat()).empty());

  cv::Mat gray(240, 320, CV_8UC1, cv::Scalar(128));
  EXPECT_TRUE(detector.Detect(gray).empty());
}

TEST(ColorDetector, ParsesColorNames)
{
  Color c = Color::kUnknown;
  EXPECT_TRUE(ColorDetector::ParseColor("red", &c));
  EXPECT_EQ(c, Color::kRed);
  EXPECT_TRUE(ColorDetector::ParseColor("GREEN", &c));
  EXPECT_EQ(c, Color::kGreen);
  EXPECT_TRUE(ColorDetector::ParseColor("Blue", &c));
  EXPECT_EQ(c, Color::kBlue);
  EXPECT_FALSE(ColorDetector::ParseColor("yellow", &c));
}

TEST(ColorDetector, TostringMatchesEnum)
{
  EXPECT_STREQ(fleet::ToString(Color::kRed), "RED");
  EXPECT_STREQ(fleet::ToString(Color::kGreen), "GREEN");
  EXPECT_STREQ(fleet::ToString(Color::kBlue), "BLUE");
  EXPECT_STREQ(fleet::ToString(Color::kUnknown), "UNKNOWN");
}
