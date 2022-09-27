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

#ifndef SPLINTER_ROS__SPLINTER1D_HPP_
#define SPLINTER_ROS__SPLINTER1D_HPP_

#include <memory>
#include <vector>

#include "common.hpp"


namespace splinter_ros
{
struct Splinter1dImpl;
class Splinter1d
{
public:
  explicit Splinter1d(
    const std::vector<double> & x,
    const std::vector<double> & y);

  explicit Splinter1d(
    const std::vector<double> & x,
    const std::vector<double> & y,
    const uint16_t & _order);

  void update(
    const std::vector<double> & x,
    const std::vector<double> & y);

  double eval(
    const double & _x,
    const FillMode & fill_mode = NO_FILL,
    const std::vector<double> & fill_value = std::vector<double>(2U, 0.0)) const;

  double evalJacobian(
    const double & _x,
    const FillMode & fill_mode = NO_FILL) const;

private:
  std::shared_ptr<Splinter1dImpl> impl_;
};
}  // namespace splinter_ros
#endif  // SPLINTER_ROS__SPLINTER1D_HPP_
