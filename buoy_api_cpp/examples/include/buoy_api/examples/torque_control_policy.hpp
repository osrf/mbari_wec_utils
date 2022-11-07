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

#ifndef BUOY_API__EXAMPLES__TORQUE_CONTROL_POLICY_HPP_
#define BUOY_API__EXAMPLES__TORQUE_CONTROL_POLICY_HPP_

/********************************************************
/ User-space to define control policy and param loading /
********************************************************/

#ifndef POLICY_ONLY
#include <buoy_api/examples/torque_controller.hpp>
#endif  // POLICY_ONLY

// interp1d for rpm->winding current
#include <simple_interp/interp1d.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <memory>
#include <vector>


struct PBTorqueControlPolicy
{
  double Torque_constant;  // N-m/Amps
  std::vector<double> N_Spec;  // RPM
  std::vector<double> Torque_Spec;  // N-m
  std::vector<double> I_Spec;  // Amps

  // interpolator for rpm -> winding current
  simple_interp::Interp1d winding_current;

  PBTorqueControlPolicy()
  : Torque_constant(0.438F),
    N_Spec{0.0F, 300.0F, 600.0F, 1000.0F, 1700.0F, 4400.0F, 6790.0F},
    Torque_Spec{0.0F, 0.0F, 0.8F, 2.9F, 5.6F, 9.8F, 16.6F},  // Matches old boost converter targets
                                                             // that have been deployed.
    I_Spec(Torque_Spec.size(), 0.0F),
    winding_current(N_Spec, I_Spec)
  {
    update_params();
  }

  void update_params()
  {
    std::transform(
      Torque_Spec.cbegin(), Torque_Spec.cend(),
      I_Spec.begin(),
      [tc = Torque_constant](const double & ts) {return ts / tc;});

    winding_current.update(N_Spec, I_Spec);
  }

  double WindingCurrentTarget(
    const double & rpm,
    const double & scale_factor,
    const double & retract_factor)
  {
    double N = fabs(rpm);
    double I = 0.0F;

    I = winding_current.eval(N);
    I *= scale_factor;

    if (rpm > 0.0F) {
      I *= -retract_factor;
    }

    return I;
  }
};

std::ostream & operator<<(std::ostream & os, const PBTorqueControlPolicy & policy)
{
  os << "PBTorqueControlPolicy:" << std::endl;

  os << "\tTorque_constant: " << policy.Torque_constant << std::endl;

  os << "\tN_Spec: " << std::flush;
  std::copy(policy.N_Spec.cbegin(), policy.N_Spec.cend(), std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tTorque_Spec: " << std::flush;
  std::copy(
    policy.Torque_Spec.cbegin(),
    policy.Torque_Spec.cend(),
    std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tI_Spec: " << std::flush;
  std::copy(policy.I_Spec.cbegin(), policy.I_Spec.cend(), std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  return os;
}

#ifndef POLICY_ONLY
void PBTorqueController::set_params()
{
  this->declare_parameter("torque_constant", policy_->Torque_constant);
  policy_->Torque_constant = this->get_parameter("torque_constant").as_double();

  this->declare_parameter(
    "n_spec", std::vector<double>(
      policy_->N_Spec.begin(),
      policy_->N_Spec.end()));
  std::vector<double> temp_double_arr = this->get_parameter("n_spec").as_double_array();
  policy_->N_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  this->declare_parameter(
    "torque_spec", std::vector<double>(
      policy_->Torque_Spec.begin(),
      policy_->Torque_Spec.end()));
  temp_double_arr = this->get_parameter("torque_spec").as_double_array();
  policy_->Torque_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  policy_->update_params();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), *policy_);
}
#endif  // POLICY_ONLY
#endif  // BUOY_API__EXAMPLES__TORQUE_CONTROL_POLICY_HPP_
