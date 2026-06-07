#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "iii_drone_core/perception/hough_transformer.hpp"

using iii_drone::perception::HoughTransformer;

TEST(HoughTransformerTest, ComputeAngleSelectsLineClosestToImageCenter)
{
  HoughTransformer transformer(nullptr);

  const int rows = 100;
  const int cols = 100;
  const float center_line_theta = static_cast<float>(M_PI / 2.0);
  const std::vector<cv::Vec2f> lines = {
    cv::Vec2f(50.0F, center_line_theta),
    cv::Vec2f(0.0F, static_cast<float>(M_PI / 4.0)),
  };

  const float angle = transformer.ComputeAngle(lines, rows, cols);

  EXPECT_NEAR(angle, center_line_theta, 1e-6F);
}
