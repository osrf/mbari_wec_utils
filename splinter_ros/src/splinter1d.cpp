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
#include <SPLINTER/utilities.h>

#include <algorithm>
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
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;

  explicit Splinter1dImpl(
    const std::vector<double> & _x,
    const std::vector<double> & _y)
  : Splinter1dImpl(_x, _y, 1U)
  {
  }

  explicit Splinter1dImpl(
    const std::vector<double> & _x,
    const std::vector<double> & _y,
    const uint16_t & _order)
  : helper(_x, _y),
    splinter1d(SPLINTER::BSpline::Builder(helper.samples)
      .degree(_order).build()),
    lower_bound(splinter1d.getDomainLowerBound()),
    upper_bound(splinter1d.getDomainUpperBound())
  {
  }

  double eval(const double & _x) const
  {
    SPLINTER::DenseVector x(1);
    x(0) = _x;
    return splinter1d.eval(x);
  }

  double evalJacobian(const double & _x) const
  {
    SPLINTER::DenseVector x(1);
    x(0) = _x;
    SPLINTER::DenseMatrix jacobian_dm = splinter1d.evalJacobian(x);
    std::vector<std::vector<double>> jacobian_vv = \
      SPLINTER::denseMatrixToVectorVector(jacobian_dm);
    return jacobian_vv[0U][0U];
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

double Splinter1d::eval(
  const double & _x,
  const FillMode & fill_mode,
  const std::vector<double> & fill_value) const
{
  assert(
    fill_value.size() == 2U &&
    "fill_value must have 2 elements: lower, upper");
  double x = _x;
  switch (fill_mode) {
    case FILL_VALUE:
      if (_x < impl_->lower_bound[0U]) {
        return fill_value[0U];
      } else if (_x > impl_->upper_bound[0U]) {
        return fill_value[1U];
      }
      break;
    case USE_BOUNDS:
      x = std::min(std::max(_x, impl_->lower_bound[0U]), impl_->upper_bound[0U]);
    case NO_FILL:
    default:
      break;
  }

  return impl_->eval(x);
}

double Splinter1d::evalJacobian(
  const double & _x,
  const FillMode & fill_mode) const
{
  double x = _x;
  switch (fill_mode) {
    case FILL_VALUE:
      std::cerr << "FILL_VALUE not implemented for Jacobian -- falling back on USE_BOUNDS" <<
        std::endl;
    case USE_BOUNDS:
      x = std::min(std::max(_x, impl_->lower_bound[0U]), impl_->upper_bound[0U]);
    case NO_FILL:
    default:
      break;
  }

  return impl_->evalJacobian(x);
}
}  // namespace splinter_ros
