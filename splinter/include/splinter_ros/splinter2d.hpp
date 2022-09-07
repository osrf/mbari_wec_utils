// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SPLINTER_ROS__SPLINTER2D_HPP_
#define SPLINTER_ROS__SPLINTER2D_HPP_

#include <memory>
#include <vector>


namespace splinter_ros
{
struct Splinter2dImpl;
class Splinter2d
{
public:
  explicit Splinter2d(const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<std::vector<double>> & z);

  void update(const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<std::vector<double>> & z);

  double eval(const double & x,
    const double & y) const;

private:
  std::shared_ptr<Splinter2dImpl> impl_;
};
}  // namespace splinter_ros
#endif  // SPLINTER_ROS__SPLINTER2D_HPP_
