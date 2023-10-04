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

#include <simple_interp/interp2d.hpp>
#include <simple_interp/interp1d.hpp>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>


namespace simple_interp
{
Interp2d::Interp2d(const std::vector<double> & x,
                   const std::vector<double> & y,
                   const std::vector<double> & z)
: x_(x),
  y_(y),
  table_{Interp2d::make_table(x, y, z)}
{
}

Interp2d::Interp2d(const table_t & table, const std::size_t & stride)
: table_{table}
{
  // x.size() == stride
  x_.reserve(stride);  // column
  y_.reserve(table.size() / stride);  // row
  for (std::size_t row = 0U; row < table.size() / stride; ++row) {
    for (std::size_t col = 0U; col < stride; ++col) {
      const point_t xyz = table_[col + stride * row];
      x_.push_back(std::get<0>(xyz));
      y_.push_back(std::get<1>(xyz));
    }
  }
}

void Interp2d::update(const std::vector<double> & x,
                      const std::vector<double> & y,
                      const std::vector<double> & z)
{
  x_ = x;
  y_ = y;
  table_ = Interp2d::make_table(x, y, z);
}

Interp2d::table_t Interp2d::make_table(
  const std::vector<double> & x,
  const std::vector<double> & y,
  const std::vector<double> & z)
{
  // row-major address into z and table is x_i (column) + x.size() * y_i (row)
  assert(x.size() * y.size() == z.size() && "x.size() * y.size() != z.size()");
  table_t table; table.reserve(z.size());
  auto z_citx = z.cbegin();
  for (auto row_citx = y.cbegin(); row_citx != y.cend(); ++row_citx) {
    for (auto col_citx = x.cbegin(); col_citx != x.cend(); ++col_citx) {
      if (z_citx != z.cend()) {
        table.push_back(std::make_tuple(*col_citx,  // x
                                        *row_citx,  // y
                                        *z_citx));  // z
      }
    }
  }
  return table;
}

// static: one-shot eval with vector data
double Interp2d::eval(
  const std::vector<double> & x_table,
  const std::vector<double> & y_table,
  const std::vector<double> & z_table,
  const double & x_eval,
  const double & y_eval)
{
  return Interp2d(x_table, y_table, z_table).eval(x_eval, y_eval);
}

// static: one-shot eval with table data
double Interp2d::eval(const Interp2d::table_t & table,
                      const std::size_t & stride,
                      const double & x,
                      const double & y)
{
  return Interp2d(table, stride).eval(x, y);
}

// operator version of eval with class instance
double Interp2d::operator()(const double & x,
                            const double & y) const
{
  return this->eval(x, y);
}

// eval with class instance
double Interp2d::eval(const double & x,
                      const double & y) const
{
  // use bounds of data table and short-cut
  bool x_oob = false;
  std::size_t x_oob_idx = 0U;
  if (x <= x_.front()) {
    x_oob_idx = 0U;
    x_oob = true;
  } else if (x >= x_.back()) {
    x_oob_idx = x_.size() - 1U;
    x_oob = true;
  }

  // check if x is in the previous bin +/- 1 and shortcut the search
  bool x_in_latest_bin{false};
  if (!x_oob && x_latest_bin_initialized_) {
    x_in_latest_bin =
      (*(x_latest_upper_edge_ - 1U) < x) &&
      (x <= *x_latest_upper_edge_);

    if (!x_in_latest_bin && x_latest_upper_edge_ + 1 != x_.cend()) {
      // try up one bin
      x_in_latest_bin =
        (*x_latest_upper_edge_ < x) &&
        (x <= *(x_latest_upper_edge_ + 1U));
      x_latest_upper_edge_ = x_latest_upper_edge_ + 1U;
    }

    if (!x_in_latest_bin && x_latest_upper_edge_ - 1U != x_.cbegin()) {
      // now try down one bin
      x_in_latest_bin =
        (*(x_latest_upper_edge_ - 2U) < x) &&
        (x <= *(x_latest_upper_edge_ - 1U));
      x_latest_upper_edge_ = x_latest_upper_edge_ - 1U;
    }
  }

  // search for correct bin otherwise shortcut with same bin
  // modified from:
  //   https://stackoverflow.com/questions/8933077/finding-which-bin-a-values-fall-into
  if (!x_oob && !x_in_latest_bin) {
    std::vector<double>::const_iterator x_citx =
      std::lower_bound(
      x_.cbegin(),
      x_.cend(),
      x);

    if (x_citx == x_.end()) {
      assert(x_citx != x_.end() && "reached end of x without finding a bin");
      return 0.0;  // shouldn't get here with bounds checking
    }

    x_latest_upper_edge_ = x_citx;
    if (!x_latest_bin_initialized_) {
      x_latest_bin_initialized_ = true;
    }
  }

  // use bounds of data table and short-cut
  bool y_oob = false;
  std::size_t y_oob_idx = 0U;
  if (y <= y_.front()) {
    y_oob_idx = 0U;
    y_oob = true;
  } else if (y >= y_.back()) {
    y_oob_idx = y_.size() - 1U;
    y_oob = true;
  }

  // check if y is in the previous bin +/- 1 and shortcut the search
  bool y_in_latest_bin{false};
  if (!y_oob && y_latest_bin_initialized_) {
    y_in_latest_bin =
      (*(y_latest_upper_edge_ - 1U) < y) &&
      (y <= *y_latest_upper_edge_);

    if (!y_in_latest_bin && y_latest_upper_edge_ + 1 != y_.cend()) {
      // try up one bin
      y_in_latest_bin =
        (*y_latest_upper_edge_ < y) &&
        (y <= *(y_latest_upper_edge_ + 1U));
      y_latest_upper_edge_ = y_latest_upper_edge_ + 1U;
    }

    if (!y_in_latest_bin && y_latest_upper_edge_ - 1U != y_.cbegin()) {
      // now try down one bin
      y_in_latest_bin =
        (*(y_latest_upper_edge_ - 2U) < y) &&
        (y <= *(y_latest_upper_edge_ - 1U));
      y_latest_upper_edge_ = y_latest_upper_edge_ - 1U;
    }
  }

  // search for correct bin otherwise shortcut with same bin
  // modified from:
  //   https://stackoverflow.com/questions/8933077/finding-which-bin-a-values-fall-into
  if (!y_oob && !y_in_latest_bin) {
    std::vector<double>::const_iterator y_citx =
      std::lower_bound(
      y_.cbegin(),
      y_.cend(),
      y);

    if (y_citx == y_.end()) {
      assert(y_citx != y_.end() && "reached end of y without finding a bin");
      return 0.0;  // shouldn't get here with bounds checking
    }

    y_latest_upper_edge_ = y_citx;
    if (!y_latest_bin_initialized_) {
      y_latest_bin_initialized_ = true;
    }
  }

  if (!x_oob && !y_oob) {
    // compute 2D linear interpolation
    const std::size_t x0_col_idx = (x_latest_upper_edge_ - 1U) - x_.cbegin();
    const std::size_t x1_col_idx = x_latest_upper_edge_ - x_.cbegin();
    const std::size_t y0_row_idx = (y_latest_upper_edge_ - 1U) - y_.cbegin();
    const std::size_t y1_row_idx = y_latest_upper_edge_ - y_.cbegin();
    const double & x0 = x_[x0_col_idx];
    const double & x1 = x_[x1_col_idx];
    const double & y0 = y_[y0_row_idx];
    const double & y1 = y_[y1_row_idx];

    const point_t & pt00 = table_[x0_col_idx + x_.size() * y0_row_idx];
    const point_t & pt10 = table_[x1_col_idx + x_.size() * y0_row_idx];
    const point_t & pt01 = table_[x0_col_idx + x_.size() * y1_row_idx];
    const point_t & pt11 = table_[x1_col_idx + x_.size() * y1_row_idx];

    const double & z00 = std::get<2>(pt00);
    const double & z10 = std::get<2>(pt10);
    const double & z01 = std::get<2>(pt01);
    const double & z11 = std::get<2>(pt11);

    const double xx = (x - x0) / (x1 - x0);
    const double yy = (y - y0) / (y1 - y0);

    return z00 * ( 1 - xx ) * ( 1 - yy ) +
           z10 * xx * ( 1 - yy ) +
           z01 * ( 1 - xx ) * yy +
           z11 * xx * yy;
  } else if (x_oob && !y_oob) {
    // compute 1D linear interpolation along y since x is on edge
    std::size_t y_row_idx = 0U;
    std::vector<double> z; z.reserve(y_.size());
    for (; y_row_idx < y_.size(); ++y_row_idx) {
      z.push_back(std::get<2>(table_[x_oob_idx + x_.size() * y_row_idx]));
    }
    return Interp1d::eval(y_, z, y);
  } else if (!x_oob && y_oob) {
    // compute 1D linear interpolation along x since y is on edge
    std::size_t x_row_idx = 0U;
    std::vector<double> z; z.reserve(x_.size());
    for (; x_row_idx < x_.size(); ++x_row_idx) {
      z.push_back(std::get<2>(table_[x_row_idx + x_.size() * y_oob_idx]));
    }
    return Interp1d::eval(x_, z, x);
  } else {
    // both x and y are on edge, so just return z
    return std::get<2>(table_[x_oob_idx + x_.size() * y_oob_idx]);
  }
}
}  // namespace simple_interp
