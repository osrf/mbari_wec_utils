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

#include <splinter_ros/splinter1d.hpp>

// Interpolation library
#include <SPLINTER/datatable.h>
#include <SPLINTER/bspline.h>
#include <SPLINTER/bsplinebuilder.h>

#include <memory>
#include <vector>


namespace splinter_ros
{
struct Splinter1dImplHelper
{
  SPLINTER::DataTable samples;

  explicit Splinter1dImplHelper(
    const std::vector<double> & _x,
    const std::vector<double> & _y)
  {
    for (size_t idx = 0U; idx < _x.size(); ++idx) {
      // Sample function at x
      SPLINTER::DenseVector x(1);
      x(0) = _x[idx];
      double y = _y[idx];

      // Store sample
      samples.addSample(x, y);
    }
  }
};

struct Splinter1dImpl
{
  Splinter1dImplHelper helper;
  SPLINTER::BSpline splinter1d;

  explicit Splinter1dImpl(
    const std::vector<double> & _x,
    const std::vector<double> & _y)
  : helper(_x, _y),
    splinter1d(SPLINTER::BSpline::Builder(helper.samples)
      .degree(1).build())
  {
  }

  double eval(const double & _x) const
  {
    SPLINTER::DenseVector x(1);
    x(0) = _x;
    return splinter1d.eval(x);
  }
};

Splinter1d::Splinter1d(
  const std::vector<double> & x,
  const std::vector<double> & y)
: impl_(std::make_shared<Splinter1dImpl>(x, y))
{
}

void Splinter1d::update(
  const std::vector<double> & x,
  const std::vector<double> & y)
{
  impl_ = std::make_shared<Splinter1dImpl>(x, y);
}

double Splinter1d::eval(const double & x) const
{
  return impl_->eval(x);
}
}  // namespace splinter_ros
