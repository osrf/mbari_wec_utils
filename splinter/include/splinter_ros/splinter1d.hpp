// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SPLINTER_ROS__SPLINTER1D_HPP_
#define SPLINTER_ROS__SPLINTER1D_HPP_

#include <memory>
#include <vector>


namespace splinter_ros
{
struct Splinter1dImpl;
class Splinter1d
{
public:
  explicit Splinter1d(const std::vector<double> & x,
    const std::vector<double> & y);

  void update(const std::vector<double> & x,
    const std::vector<double> & y);

  double eval(const double & x) const;

private:
  std::shared_ptr<Splinter1dImpl> impl_;
};
}  // namespace splinter_ros
#endif  // SPLINTER_ROS__SPLINTER1D_HPP_
