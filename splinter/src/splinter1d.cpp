// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <splinter_ros/splinter1d.hpp>

// Interpolation library
#include <splinter/datatable.h>
#include <splinter/bspline.h>
#include <splinter/bsplinebuilder.h>

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
