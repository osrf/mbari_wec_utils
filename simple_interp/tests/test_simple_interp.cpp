// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <vector>

#include <simple_interp/interp1d.hpp>
#include <simple_interp/interp2d.hpp>


// NOLINTNEXTLINE
class SimpleInterp1dTest : public ::testing::Test
{
public:
  const std::vector<double> x;
  const std::vector<double> y;
  simple_interp::Interp1d interp1d;

  SimpleInterp1dTest()
  : x{0.0, 1.0, 2.0},
    y{-1.0, 0.0, 1.0},
    interp1d(x, y)
  {
  }
};


TEST_F(SimpleInterp1dTest, Interp1d)
{
  EXPECT_EQ(-1.0, interp1d(0.0));
  EXPECT_EQ(-1.0, interp1d(-1.0));  // x oob to the left

  EXPECT_EQ(-0.5, interp1d(0.5));

  EXPECT_EQ(0.0, interp1d(1.0));

  EXPECT_EQ(0.5, interp1d(1.5));

  EXPECT_EQ(1.0, interp1d(2.0));
  EXPECT_EQ(1.0, interp1d(3.0));  // x oob to the right
}


// NOLINTNEXTLINE
class SimpleInterp2dTest : public ::testing::Test
{
public:
  const std::vector<double> x;
  const std::vector<double> y;
  const std::vector<double> z;
  simple_interp::Interp2d interp2d;

  SimpleInterp2dTest()
  : x{0.0, 1.0, 2.0},
    y{-1.0, 0.0, 1.0},
    z{-4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0},
    interp2d(x, y, z)
  {
  }
};


TEST_F(SimpleInterp2dTest, Interp2d)
{
  EXPECT_EQ(-4.0, interp2d(-1.0, -1.0));  // x oob to the left
  EXPECT_EQ(-4.0, interp2d(0.0, -2.0));  // y oob to the left
  EXPECT_EQ(-4.0, interp2d(-1.0, -2.0));  // x,y oob to the left
  EXPECT_EQ(-4.0, interp2d(0.0, -1.0));
  EXPECT_EQ(-3.0, interp2d(1.0, -1.0));
  EXPECT_EQ(-2.0, interp2d(2.0, -1.0));
  EXPECT_EQ(-2.0, interp2d(3.0, -1.0));  // x oob to the right
  EXPECT_EQ(-2.0, interp2d(2.0, -2.0));  // y oob to the left
  EXPECT_EQ(-2.0, interp2d(3.0, -2.0));  // x,y oob to the right,left

  EXPECT_EQ(-1.0, interp2d(-1.0, 0.0));  // x oob to the left
  EXPECT_EQ(-1.0, interp2d(0.0, 0.0));
  EXPECT_EQ(0.0, interp2d(1.0, 0.0));
  EXPECT_EQ(1.0, interp2d(2.0, 0.0));
  EXPECT_EQ(1.0, interp2d(3.0, 0.0));  // x oob to the right

  EXPECT_EQ(2.0, interp2d(-1.0, 1.0));  // x oob to the left
  EXPECT_EQ(2.0, interp2d(0.0, 2.0));  // y oob to the right
  EXPECT_EQ(2.0, interp2d(-1.0, 2.0));  // x oob to the left,right
  EXPECT_EQ(2.0, interp2d(0.0, 1.0));
  EXPECT_EQ(3.0, interp2d(1.0, 1.0));
  EXPECT_EQ(4.0, interp2d(2.0, 1.0));
  EXPECT_EQ(4.0, interp2d(3.0, 1.0));  // x oob to the right
  EXPECT_EQ(4.0, interp2d(2.0, 2.0));  // y oob to the right
  EXPECT_EQ(4.0, interp2d(3.0, 2.0));  // x,y oob to the right

  EXPECT_EQ(-2.0, interp2d(0.5, -0.5));
  EXPECT_EQ(-2.5, interp2d(0.0, -0.5));
  EXPECT_EQ(-2.5, interp2d(-1.0, -0.5));  // x oob and reduces to z = interp1d(y)
  EXPECT_EQ(-3.5, interp2d(0.5, -1.0));
  EXPECT_EQ(-3.5, interp2d(0.5, -2.0));  // y oob and reduces to z = interp1d(x)
}
