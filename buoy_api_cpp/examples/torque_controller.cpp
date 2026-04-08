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

#include <buoy_api/examples/torque_control_policy.hpp>
#include <buoy_api/examples/torque_controller.hpp>

#include <string>
#include <memory>

#include <buoy_interfaces/srv/inc_wave_height.hpp>


PBTorqueController::PBTorqueController(const std::string & node_name)
: buoy_api::Interface<PBTorqueController>(node_name)
{
  policy_.reset(new PBTorqueControlPolicy());
  set_params();

  set_pc_pack_rate();
}

void PBTorqueController::power_callback(const buoy_interfaces::msg::PCRecord & data)
{
  float wind_curr = policy_->WindingCurrentTarget(data.rpm, data.scale, data.retract);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      this->get_name()),
    "WindingCurrent: f(" << data.rpm << ", " << data.scale << ", " << data.retract << ") = " <<
      wind_curr
  );

  auto future = this->send_pc_wind_curr_command(wind_curr);

  /*
  // Example using the full API
  auto future = this->send_pc_wind_curr_command(
    wind_curr,  // command value
    true,  // blocking
    0.1,  // timeout in seconds
    true  // use default callback
  );
  if (future.valid())
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        this->get_name()),
      "Got valid pc wind curr command response: " << static_cast<int>(future.get()->result.value)
    );
  }
  else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        this->get_name()),
      "pc wind curr command did not complete within timeout"
    );
  }
  */
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<PBTorqueController>("pb_torque_controller");
  controller->spin();
  rclcpp::shutdown();

  return 0;
}
