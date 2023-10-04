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

#ifndef SIMPLE_INTERP__INTERP2D_HPP_
#define SIMPLE_INTERP__INTERP2D_HPP_


#include <tuple>
#include <utility>
#include <vector>


namespace simple_interp
{
class Interp2d
{
public:
  //                   x       y       z
  typedef std::tuple<double, double, double> point_t;
  typedef std::vector<point_t> table_t;

  Interp2d(
    const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<double> & z);
  explicit Interp2d(
    const table_t & table,
    const std::size_t & stride);

  void update(
    const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<double> & z);

  static table_t make_table(
    const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<double> & z);

  // one-shot eval with vector data
  static double eval(
    const std::vector<double> & x_table,
    const std::vector<double> & y_table,
    const std::vector<double> & z_table,
    const double & x_eval,
    const double & y_eval);
  // one-shot eval with table data
  static double eval(
    const table_t & table,
    const std::size_t & stride,
    const double & x,
    const double & y);
  // operator version of eval with class instance
  double operator()(const double & x, const double & y) const;
  // eval with class instance
  double eval(const double & x, const double & y) const;

private:
  std::vector<double> x_, y_;
  table_t table_;
  mutable std::vector<double>::const_iterator x_latest_upper_edge_;
  mutable std::vector<double>::const_iterator y_latest_upper_edge_;
  mutable bool x_latest_bin_initialized_{false};
  mutable bool y_latest_bin_initialized_{false};
};
}  // namespace simple_interp

#endif  // SIMPLE_INTERP__INTERP2D_HPP_
