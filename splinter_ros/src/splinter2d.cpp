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

#include <splinter_ros/splinter2d.hpp>

// Interpolation library
#include <SPLINTER/datatable.h>
#include <SPLINTER/bspline.h>
#include <SPLINTER/bsplinebuilder.h>

#include <memory>
#include <vector>


namespace splinter_ros
{
struct Splinter2dImplHelper
{
  SPLINTER::DataTable samples;

  explicit Splinter2dImplHelper(
    const std::vector<double> & _x,
    const std::vector<double> & _y,
    const std::vector<std::vector<double>> & _z)
  {
    for (size_t idx = 0U; idx < _x.size(); ++idx) {
      for (size_t jdx = 0U; jdx < _y.size(); ++jdx) {
        // Sample function at x
        SPLINTER::DenseVector x(2);
        x(0) = _x[idx];
        x(1) = _y[jdx];
        double y = _z[jdx][idx];

        // Store sample
        samples.addSample(x, y);
      }
    }
  }
};

struct Splinter2dImpl
{
  Splinter2dImplHelper helper;
  SPLINTER::BSpline splinter2d;

  explicit Splinter2dImpl(
    const std::vector<double> & _x,
    const std::vector<double> & _y,
    const std::vector<std::vector<double>> & _z)
  : helper(_x, _y, _z),
    splinter2d(SPLINTER::BSpline::Builder(helper.samples)
      .degree(1).build())
  {
  }

  double eval(
    const double & _x,
    const double & _y) const
  {
    SPLINTER::DenseVector x(2);
    x(0) = _x;
    x(1) = _y;
    return splinter2d.eval(x);
  }
};

Splinter2d::Splinter2d(
  const std::vector<double> & x,
  const std::vector<double> & y,
  const std::vector<std::vector<double>> & z)
: impl_(std::make_shared<Splinter2dImpl>(x, y, z))
{
}

void Splinter2d::update(
  const std::vector<double> & x,
  const std::vector<double> & y,
  const std::vector<std::vector<double>> & z)
{
  impl_ = std::make_shared<Splinter2dImpl>(x, y, z);
}

double Splinter2d::eval(
  const double & x,
  const double & y) const
{
  return impl_->eval(x, y);
}
}  // namespace splinter_ros
