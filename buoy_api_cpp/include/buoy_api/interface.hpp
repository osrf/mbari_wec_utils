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

#ifndef BUOY_API__INTERFACE_HPP_
#define BUOY_API__INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

// Pack Rate Params
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/parameter.hpp>

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

// pbsrv commands
// power microcontroller
#include "buoy_interfaces/srv/pc_batt_switch_command.hpp"
#include "buoy_interfaces/srv/pc_bias_curr_command.hpp"
#include "buoy_interfaces/srv/pc_charge_curr_lim_command.hpp"
#include "buoy_interfaces/srv/pc_draw_curr_lim_command.hpp"
#include "buoy_interfaces/srv/pc_pack_rate_command.hpp"
#include "buoy_interfaces/srv/pc_retract_command.hpp"
#include "buoy_interfaces/srv/pc_scale_command.hpp"
#include "buoy_interfaces/srv/pc_std_dev_targ_command.hpp"
#include "buoy_interfaces/srv/pcv_targ_max_command.hpp"
#include "buoy_interfaces/srv/pc_wind_curr_command.hpp"
#include "buoy_interfaces/srv/gain_command.hpp"

// battery microcontroller
#include "buoy_interfaces/srv/bc_reset_command.hpp"

// spring microcontroller
#include "buoy_interfaces/srv/sc_pack_rate_command.hpp"
#include "buoy_interfaces/srv/sc_reset_command.hpp"
#include "buoy_interfaces/srv/valve_command.hpp"
#include "buoy_interfaces/srv/pump_command.hpp"
#include "buoy_interfaces/srv/bender_command.hpp"
#include "buoy_interfaces/srv/tether_command.hpp"

// trefoil microcontroller
#include "buoy_interfaces/srv/tf_reset_command.hpp"
#include "buoy_interfaces/srv/tf_set_actual_pos_command.hpp"
#include "buoy_interfaces/srv/tf_set_charge_mode_command.hpp"
#include "buoy_interfaces/srv/tf_set_curr_lim_command.hpp"
#include "buoy_interfaces/srv/tf_set_mode_command.hpp"
#include "buoy_interfaces/srv/tf_set_pos_command.hpp"
#include "buoy_interfaces/srv/tf_set_state_machine_command.hpp"
#include "buoy_interfaces/srv/tf_watch_dog_command.hpp"

// pb telemetry
#include "buoy_interfaces/msg/xb_record.hpp"  // ahrs
#include "buoy_interfaces/msg/bc_record.hpp"  // battery
#include "buoy_interfaces/msg/sc_record.hpp"  // spring
#include "buoy_interfaces/msg/pc_record.hpp"  // power
#include "buoy_interfaces/msg/tf_record.hpp"  // trefoil
#include "buoy_interfaces/msg/pb_record.hpp"  // consolidated


namespace buoy_api
{
using std::placeholders::_1;

std::map<int8_t, std::string> pbsrv_enum2str = {{0, "OK"},
  {-1, "BAD_SOCK"},
  {-2, "BAD_OPTS"},
  {-3, "BAD_INPUT"}};


template<class ControllerImplCRTP>
class Interface : public rclcpp::Node
{
public:
  using CRTP = Interface;  // syntactic sugar for friend class
                           // see https://stackoverflow.com/a/58435857/9686600
  explicit Interface(const std::string & node_name)
  : Interface(node_name, false)
  {
  }

  explicit Interface(const std::string & node_name, bool _wait_for_services)
  : Node(node_name)
  {
    pc_pack_rate_param_client_ =
      std::make_unique<rclcpp::SyncParametersClient>(
      std::shared_ptr<rclcpp::Node>(
        static_cast<ControllerImplCRTP *>(this),
        [](rclcpp::Node *) {}),
      "/power_controller");
    pc_pack_rate_client_ =
      this->create_client<buoy_interfaces::srv::PCPackRateCommand>("/pc_pack_rate_command");
    pc_wind_curr_client_ =
      this->create_client<buoy_interfaces::srv::PCWindCurrCommand>("/pc_wind_curr_command");
    bender_client_ = this->create_client<buoy_interfaces::srv::BenderCommand>("/bender_command");
    bc_reset_client_ =
      this->create_client<buoy_interfaces::srv::BCResetCommand>("/bc_reset_command");
    pump_client_ = this->create_client<buoy_interfaces::srv::PumpCommand>("/pump_command");
    valve_client_ = this->create_client<buoy_interfaces::srv::ValveCommand>("/valve_command");
    tether_client_ = this->create_client<buoy_interfaces::srv::TetherCommand>("/tether_command");
    sc_reset_client_ =
      this->create_client<buoy_interfaces::srv::SCResetCommand>("/sc_reset_command");
    sc_pack_rate_param_client_ =
      std::make_unique<rclcpp::SyncParametersClient>(
      std::shared_ptr<rclcpp::Node>(
        static_cast<ControllerImplCRTP *>(this),
        [](rclcpp::Node *) {}),
      "/spring_controller");
    sc_pack_rate_client_ =
      this->create_client<buoy_interfaces::srv::SCPackRateCommand>("/sc_pack_rate_command");
    pc_scale_client_ =
      this->create_client<buoy_interfaces::srv::PCScaleCommand>("/pc_scale_command");
    pc_retract_client_ =
      this->create_client<buoy_interfaces::srv::PCRetractCommand>("/pc_retract_command");
    pc_v_targ_max_client_ =
      this->create_client<buoy_interfaces::srv::PCVTargMaxCommand>("/pc_v_targ_max_command");
    pc_charge_curr_lim_client_ =
      this->create_client<buoy_interfaces::srv::PCChargeCurrLimCommand>(
      "/pc_charge_curr_lim_command");
    pc_batt_switch_client_ =
      this->create_client<buoy_interfaces::srv::PCBattSwitchCommand>("/pc_batt_switch_command");
    gain_client_ = this->create_client<buoy_interfaces::srv::GainCommand>("/gain_command");
    pc_std_dev_targ_client_ =
      this->create_client<buoy_interfaces::srv::PCStdDevTargCommand>("/pc_std_dev_targ_command");
    pc_draw_curr_lim_client_ =
      this->create_client<buoy_interfaces::srv::PCDrawCurrLimCommand>("/pc_draw_curr_lim_command");
    pc_bias_curr_client_ =
      this->create_client<buoy_interfaces::srv::PCBiasCurrCommand>("/pc_bias_curr_command");
    tf_set_pos_client_ =
      this->create_client<buoy_interfaces::srv::TFSetPosCommand>("/tf_set_pos_command");
    tf_set_actual_pos_client_ =
      this->create_client<buoy_interfaces::srv::TFSetActualPosCommand>(
      "/tf_set_actual_pos_command");
    tf_set_mode_client_ =
      this->create_client<buoy_interfaces::srv::TFSetModeCommand>("/tf_set_mode_command");
    tf_set_charge_mode_client_ =
      this->create_client<buoy_interfaces::srv::TFSetChargeModeCommand>(
      "/tf_set_charge_mode_command");
    tf_set_curr_lim_client_ =
      this->create_client<buoy_interfaces::srv::TFSetCurrLimCommand>("/tf_set_curr_lim_command");
    tf_set_state_machine_client_ =
      this->create_client<buoy_interfaces::srv::TFSetStateMachineCommand>(
      "/tf_set_state_machine_command");
    tf_watch_dog_client_ =
      this->create_client<buoy_interfaces::srv::TFWatchDogCommand>("/tf_watch_dog_command");
    tf_reset_client_ =
      this->create_client<buoy_interfaces::srv::TFResetCommand>("/tf_reset_command");

    setup_subscribers();
    bool found = false;
    do {
      found = wait_for_services();
    } while (rclcpp::ok() && !found && _wait_for_services);
  }

  bool wait_for_services()
  {
    bool found_pc_param = wait_for_service(
      pc_pack_rate_param_client_,
      "/power_controller/set_parameters");
    bool found_pc_packrate = wait_for_service(pc_pack_rate_client_, "/pc_pack_rate_command");
    bool found = found_pc_param || found_pc_packrate;
    found &= wait_for_service(pc_wind_curr_client_, "/pc_wind_curr_command");
    found &= wait_for_service(bender_client_, "/bender_command");
    found &= wait_for_service(bc_reset_client_, "/bc_reset_command");
    found &= wait_for_service(pump_client_, "/pump_command");
    found &= wait_for_service(valve_client_, "/valve_command");
    found &= wait_for_service(tether_client_, "/tether_command");
    found &= wait_for_service(sc_reset_client_, "/sc_reset_command");
    bool found_sc_param = wait_for_service(
      sc_pack_rate_param_client_,
      "/spring_controller/set_parameters");
    bool found_sc_packrate = wait_for_service(sc_pack_rate_client_, "/sc_pack_rate_command");
    found &= found_sc_param || found_sc_packrate;
    found &= wait_for_service(pc_scale_client_, "/pc_scale_command");
    found &= wait_for_service(pc_retract_client_, "/pc_retract_command");
    found &= wait_for_service(pc_v_targ_max_client_, "/pc_v_targ_max_command");
    found &= wait_for_service(pc_charge_curr_lim_client_, "/pc_charge_curr_lim_command");
    found &= wait_for_service(pc_batt_switch_client_, "/pc_batt_switch_command");
    found &= wait_for_service(gain_client_, "/gain_command");
    found &= wait_for_service(pc_std_dev_targ_client_, "/pc_std_dev_targ_command");
    found &= wait_for_service(pc_draw_curr_lim_client_, "/pc_draw_curr_lim_command");
    found &= wait_for_service(pc_bias_curr_client_, "/pc_bias_curr_command");
    found &= wait_for_service(tf_set_pos_client_, "/tf_set_pos_command");
    found &= wait_for_service(tf_set_actual_pos_client_, "/tf_set_actual_pos_command");
    found &= wait_for_service(tf_set_mode_client_, "/tf_set_mode_command");
    found &= wait_for_service(tf_set_charge_mode_client_, "/tf_set_charge_mode_command");
    found &= wait_for_service(tf_set_curr_lim_client_, "/tf_set_curr_lim_command");
    found &= wait_for_service(tf_set_state_machine_client_, "/tf_set_state_machine_command");
    found &= wait_for_service(tf_watch_dog_client_, "/tf_watch_dog_command");
    found &= wait_for_service(tf_reset_client_, "/tf_reset_command");

    if (!found) {
      RCLCPP_ERROR(rclcpp::get_logger(this->get_name()), "Did not find required services");
    }
    return found;
  }

  // if user has shadowed a callback in their user-derived class, this will use their
  // implementation; if they did not define one, the subscriber will not be set up
  void setup_subscribers()
  {
    if (&ControllerImplCRTP::ahrs_callback == &Interface::ahrs_callback) {
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to XBRecord on '/ahrs_data' and '/xb_record'");
      ahrs_data_sub_ = this->create_subscription<buoy_interfaces::msg::XBRecord>(
        "/ahrs_data", 1,
        std::bind(
          &ControllerImplCRTP::ahrs_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
      xb_record_sub_ = this->create_subscription<buoy_interfaces::msg::XBRecord>(
        "/xb_record", 1,
        std::bind(
          &ControllerImplCRTP::ahrs_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
    }

    if (&ControllerImplCRTP::battery_callback == &Interface::battery_callback) {
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to BCRecord on '/battery_data' and '/bc_record'");
      battery_data_sub_ = this->create_subscription<buoy_interfaces::msg::BCRecord>(
        "/battery_data", 1,
        std::bind(
          &ControllerImplCRTP::battery_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
      bc_record_sub_ = this->create_subscription<buoy_interfaces::msg::BCRecord>(
        "/bc_record", 1,
        std::bind(
          &ControllerImplCRTP::battery_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
    }

    if (&ControllerImplCRTP::spring_callback == &Interface::spring_callback) {
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to SCRecord on '/spring_data' and '/sc_record'");
      spring_data_sub_ = this->create_subscription<buoy_interfaces::msg::SCRecord>(
        "/spring_data", 1,
        std::bind(
          &ControllerImplCRTP::spring_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
      sc_record_sub_ = this->create_subscription<buoy_interfaces::msg::SCRecord>(
        "/sc_record", 1,
        std::bind(
          &ControllerImplCRTP::spring_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
    }

    if (&ControllerImplCRTP::power_callback == &Interface::power_callback) {
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to PCRecord on '/power_data' and '/pc_record'");
      power_data_sub_ = this->create_subscription<buoy_interfaces::msg::PCRecord>(
        "/power_data", 1,
        std::bind(
          &ControllerImplCRTP::power_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
      pc_record_sub_ = this->create_subscription<buoy_interfaces::msg::PCRecord>(
        "/pc_record", 1,
        std::bind(
          &ControllerImplCRTP::power_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
    }

    if (&ControllerImplCRTP::trefoil_callback == &Interface::trefoil_callback) {
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to TFRecord on '/trefoil_data' and '/tf_record'");
      trefoil_data_sub_ = this->create_subscription<buoy_interfaces::msg::TFRecord>(
        "/trefoil_data", 1,
        std::bind(
          &ControllerImplCRTP::trefoil_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
      tf_record_sub_ = this->create_subscription<buoy_interfaces::msg::TFRecord>(
        "/tf_record", 1,
        std::bind(
          &ControllerImplCRTP::trefoil_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
    }

    if (&ControllerImplCRTP::powerbuoy_callback == &Interface::powerbuoy_callback) {
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to PBRecord on '/powerbuoy_data'");
      powerbuoy_data_sub_ =
        this->create_subscription<buoy_interfaces::msg::PBRecord>(
        "/powerbuoy_data", 1,
        std::bind(
          &ControllerImplCRTP::powerbuoy_callback,
          static_cast<ControllerImplCRTP *>(this), _1));
    }
  }

  // set publish rate of PC Microcontroller telemetry
  void set_pc_pack_rate(const uint8_t & rate_hz = 50)
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCPackRateCommand::Request>();
    request->rate_hz = rate_hz;

    PCPackRateServiceCallback pc_pack_rate_callback =
      default_service_response_callback<PCPackRateServiceCallback,
        PCPackRateServiceResponseFuture>();

    auto response = pc_pack_rate_client_->async_send_request(request, pc_pack_rate_callback);
  }

  // set publish rate of SC Microcontroller telemetry
  void set_sc_pack_rate(const uint8_t & rate_hz = 50)
  {
    auto request = std::make_shared<buoy_interfaces::srv::SCPackRateCommand::Request>();
    request->rate_hz = rate_hz;

    SCPackRateServiceCallback sc_pack_rate_callback =
      default_service_response_callback<SCPackRateServiceCallback,
        SCPackRateServiceResponseFuture>();

    auto response = sc_pack_rate_client_->async_send_request(request, sc_pack_rate_callback);
  }

  // set publish rate of PC Microcontroller telemetry
  void set_pc_pack_rate_param(const double & rate_hz = 50.0)
  {
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter{"publish_rate",
        rclcpp::ParameterValue{rate_hz}}};
    auto result = pc_pack_rate_param_client_->set_parameters(params);
    if (result[0U].successful) {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Successfully set publish_rate for power_controller");
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Failed to set publish_rate for power_controller: " << result[0U].reason);
    }
  }

  // set publish rate of SC Microcontroller telemetry
  void set_sc_pack_rate_param(const double & rate_hz = 50.0)
  {
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter{"publish_rate",
        rclcpp::ParameterValue{rate_hz}}};
    auto result = sc_pack_rate_param_client_->set_parameters(params);
    if (result[0U].successful) {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Successfully set publish_rate for spring_controller");
    } else {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Failed to set publish_rate for spring_controller: " << result[0U].reason);
    }
  }

  // abbrv futures
  using BenderServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::BenderCommand>::SharedFuture;
  using BCResetServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::BCResetCommand>::SharedFuture;
  using PumpServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PumpCommand>::SharedFuture;
  using ValveServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::ValveCommand>::SharedFuture;
  using TetherServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TetherCommand>::SharedFuture;
  using SCResetServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::SCResetCommand>::SharedFuture;
  using SCPackRateServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::SCPackRateCommand>::SharedFuture;
  using PCScaleServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCScaleCommand>::SharedFuture;
  using PCRetractServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCRetractCommand>::SharedFuture;
  using PCVTargMaxServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCVTargMaxCommand>::SharedFuture;
  using PCChargeCurrLimServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCChargeCurrLimCommand>::SharedFuture;
  using PCBattSwitchServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCBattSwitchCommand>::SharedFuture;
  using GainServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::GainCommand>::SharedFuture;
  using PCStdDevTargServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCStdDevTargCommand>::SharedFuture;
  using PCDrawCurrLimServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCDrawCurrLimCommand>::SharedFuture;
  using PCWindCurrServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCWindCurrCommand>::SharedFuture;
  using PCBiasCurrServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCBiasCurrCommand>::SharedFuture;
  using PCPackRateServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::PCPackRateCommand>::SharedFuture;
  using TFSetPosServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFSetPosCommand>::SharedFuture;
  using TFSetActualPosServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFSetActualPosCommand>::SharedFuture;
  using TFSetModeServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFSetModeCommand>::SharedFuture;
  using TFSetChargeModeServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFSetChargeModeCommand>::SharedFuture;
  using TFSetCurrLimServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFSetCurrLimCommand>::SharedFuture;
  using TFSetStateMachineServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFSetStateMachineCommand>::SharedFuture;
  using TFWatchDogServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFWatchDogCommand>::SharedFuture;
  using TFResetServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::TFResetCommand>::SharedFuture;

  ValveServiceResponseFuture send_valve_command(const uint16_t & duration_sec)
  {
    auto request = std::make_shared<buoy_interfaces::srv::ValveCommand::Request>();
    request->duration_sec = duration_sec;

    ValveServiceResponseFuture valve_response_future =
      valve_client_->async_send_request(request);

    return valve_response_future;
  }

  PumpServiceResponseFuture send_pump_command(const uint16_t & duration_sec)
  {
    auto request = std::make_shared<buoy_interfaces::srv::PumpCommand::Request>();
    request->duration_sec = duration_sec;

    PumpServiceResponseFuture pump_response_future =
      pump_client_->async_send_request(request);

    return pump_response_future;
  }

  PCWindCurrServiceResponseFuture send_pc_wind_curr_command(const float & wind_curr)
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCWindCurrCommand::Request>();
    request->wind_curr = wind_curr;

    PCWindCurrServiceResponseFuture pc_wind_curr_response_future =
      pc_wind_curr_client_->async_send_request(request);

    return pc_wind_curr_response_future;
  }

  PCBiasCurrServiceResponseFuture send_pc_bias_curr_command(const float & bias_curr)
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCBiasCurrCommand::Request>();
    request->bias_curr = bias_curr;

    PCBiasCurrServiceResponseFuture pc_bias_curr_response_future =
      pc_bias_curr_client_->async_send_request(request);

    return pc_bias_curr_response_future;
  }

  PCScaleServiceResponseFuture send_pc_scale_command(const float & scale)
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCScaleCommand::Request>();
    request->scale = scale;

    PCScaleServiceResponseFuture pc_scale_response_future =
      pc_scale_client_->async_send_request(request);

    return pc_scale_response_future;
  }

  PCRetractServiceResponseFuture send_pc_retract_command(const float & retract)
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCRetractCommand::Request>();
    request->retract = retract;

    PCRetractServiceResponseFuture pc_retract_response_future =
      pc_retract_client_->async_send_request(request);

    return pc_retract_response_future;
  }

protected:
  virtual ~Interface() = default;

  // set_params and callbacks optionally defined by user
  virtual void set_params() {}
  void ahrs_callback(const buoy_interfaces::msg::XBRecord &) {}
  void battery_callback(const buoy_interfaces::msg::BCRecord &) {}
  void spring_callback(const buoy_interfaces::msg::SCRecord &) {}
  void power_callback(const buoy_interfaces::msg::PCRecord &) {}
  void trefoil_callback(const buoy_interfaces::msg::TFRecord &) {}
  void powerbuoy_callback(const buoy_interfaces::msg::PBRecord &) {}

  // abbrv callback types
  using BenderServiceCallback = rclcpp::Client<buoy_interfaces::srv::BenderCommand>::CallbackType;
  using BCResetServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::BCResetCommand>::CallbackType;
  using PumpServiceCallback = rclcpp::Client<buoy_interfaces::srv::PumpCommand>::CallbackType;
  using ValveServiceCallback = rclcpp::Client<buoy_interfaces::srv::ValveCommand>::CallbackType;
  using TetherServiceCallback = rclcpp::Client<buoy_interfaces::srv::TetherCommand>::CallbackType;
  using SCResetServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::SCResetCommand>::CallbackType;
  using SCPackRateServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::SCPackRateCommand>::CallbackType;
  using PCScaleServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCScaleCommand>::CallbackType;
  using PCRetractServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCRetractCommand>::CallbackType;
  using PCVTargMaxServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCVTargMaxCommand>::CallbackType;
  using PCChargeCurrLimServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCChargeCurrLimCommand>::CallbackType;
  using PCBattSwitchServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCBattSwitchCommand>::CallbackType;
  using GainServiceCallback = rclcpp::Client<buoy_interfaces::srv::GainCommand>::CallbackType;
  using PCStdDevTargServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCStdDevTargCommand>::CallbackType;
  using PCDrawCurrLimServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCDrawCurrLimCommand>::CallbackType;
  using PCWindCurrServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCWindCurrCommand>::CallbackType;
  using PCBiasCurrServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCBiasCurrCommand>::CallbackType;
  using PCPackRateServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::PCPackRateCommand>::CallbackType;
  using TFSetPosServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFSetPosCommand>::CallbackType;
  using TFSetActualPosServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFSetActualPosCommand>::CallbackType;
  using TFSetModeServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFSetModeCommand>::CallbackType;
  using TFSetChargeModeServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFSetChargeModeCommand>::CallbackType;
  using TFSetCurrLimServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFSetCurrLimCommand>::CallbackType;
  using TFSetStateMachineServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFSetStateMachineCommand>::CallbackType;
  using TFWatchDogServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFWatchDogCommand>::CallbackType;
  using TFResetServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::TFResetCommand>::CallbackType;

  /***** Example Default Callback Usage *****
   * BenderServiceCallback bender_callback =
   *   default_service_response_callback<BenderServiceCallback,
   *     BenderServiceResponseFuture>();
   * BCResetServiceCallback bc_reset_callback =
   *   default_service_response_callback<BCResetServiceCallback,
   *     BCResetServiceResponseFuture>();
   * PumpServiceCallback pump_callback =
   *   default_service_response_callback<PumpServiceCallback,
   *     PumpServiceResponseFuture>();
   * ValveServiceCallback valve_callback =
   *   default_service_response_callback<ValveServiceCallback,
   *     ValveServiceResponseFuture>();
   * TetherServiceCallback tether_callback =
   *   default_service_response_callback<TetherServiceCallback,
   *     TetherServiceResponseFuture>();
   * SCResetServiceCallback sc_reset_callback =
   *   default_service_response_callback<SCResetServiceCallback,
   *     SCResetServiceResponseFuture>();
   * SCPackRateServiceCallback sc_pack_rate_callback =
   *   default_service_response_callback<SCPackRateServiceCallback,
   *     SCPackRateServiceResponseFuture>();
   * PCScaleServiceCallback pc_scale_callback =
   *   default_service_response_callback<PCScaleServiceCallback,
   *     PCScaleServiceResponseFuture>();
   * PCRetractServiceCallback pc_retract_callback =
   *   default_service_response_callback<PCRetractServiceCallback,
   *     PCRetractServiceResponseFuture>();
   * PCVTargMaxServiceCallback pc_v_targ_max_callback =
   *   default_service_response_callback<PCVTargMaxServiceCallback,
   *     PCVTargMaxServiceResponseFuture>();
   * PCChargeCurrLimServiceCallback pc_charge_curr_lim_callback =
   *   default_service_response_callback<PCChargeCurrLimServiceCallback,
   *     PCChargeCurrLimServiceResponseFuture>();
   * PCBattSwitchServiceCallback pc_batt_switch_callback =
   *   default_service_response_callback<PCBattSwitchServiceCallback,
   *     PCBattSwitchServiceResponseFuture>();
   * GainServiceCallback gain_callback =
   *   default_service_response_callback<GainServiceCallback,
   *     GainServiceResponseFuture>();
   * PCStdDevTargServiceCallback pc_std_dev_targ_callback =
   *   default_service_response_callback<PCStdDevTargServiceCallback,
   *     PCStdDevTargServiceResponseFuture>();
   * PCDrawCurrLimServiceCallback pc_draw_curr_lim_callback =
   *   default_service_response_callback<PCDrawCurrLimServiceCallback,
   *     PCDrawCurrLimServiceResponseFuture>();
   * PCWindCurrServiceCallback pc_wind_curr_callback =
   *   default_service_response_callback<PCWindCurrServiceCallback,
   *     PCWindCurrServiceResponseFuture>();
   * PCBiasCurrServiceCallback pc_bias_curr_callback =
   *   default_service_response_callback<PCBiasCurrServiceCallback,
   *     PCBiasCurrServiceResponseFuture>();
   * PCPackRateServiceCallback pc_pack_rate_callback =
   *   default_service_response_callback<PCPackRateServiceCallback,
   *     PCPackRateServiceResponseFuture>();
   * TFSetPosServiceCallback tf_set_pos_callback =
   *   default_service_response_callback<TFSetPosServiceCallback,
   *     TFSetPosServiceResponseFuture>();
   * TFSetActualPosServiceCallback tf_set_actual_pos_callback =
   *   default_service_response_callback<TFSetActualPosServiceCallback,
   *     TFSetActualPosServiceResponseFuture>();
   * TFSetModeServiceCallback tf_set_mode_callback =
   *   default_service_response_callback<TFSetModeServiceCallback,
   *     TFSetModeServiceResponseFuture>();
   * TFSetChargeModeServiceCallback tf_set_charge_mode_callback =
   *   default_service_response_callback<TFSetChargeModeServiceCallback,
   *     TFSetChargeModeServiceResponseFuture>();
   * TFSetCurrLimServiceCallback tf_set_curr_lim_callback =
   *   default_service_response_callback<TFSetCurrLimServiceCallback,
   *     TFSetCurrLimServiceResponseFuture>();
   * TFSetStateMachineServiceCallback tf_set_state_machine_callback =
   *   default_service_response_callback<TFSetStateMachineServiceCallback,
   *     TFSetStateMachineServiceResponseFuture>();
   * TFWatchDogServiceCallback tf_watchdog_callback =
   *   default_service_response_callback<TFWatchDogServiceCallback,
   *     TFWatchDogServiceResponseFuture>();
   * TFResetServiceCallback tf_reset_callback =
   *   default_service_response_callback<TFResetServiceCallback,
   *     TFResetServiceResponseFuture>();
   */

  // declare all clients
  rclcpp::Client<buoy_interfaces::srv::BenderCommand>::SharedPtr bender_client_;
  rclcpp::Client<buoy_interfaces::srv::BCResetCommand>::SharedPtr bc_reset_client_;
  rclcpp::Client<buoy_interfaces::srv::PumpCommand>::SharedPtr pump_client_;
  rclcpp::Client<buoy_interfaces::srv::ValveCommand>::SharedPtr valve_client_;
  rclcpp::Client<buoy_interfaces::srv::TetherCommand>::SharedPtr tether_client_;
  rclcpp::Client<buoy_interfaces::srv::SCResetCommand>::SharedPtr sc_reset_client_;
  rclcpp::Client<buoy_interfaces::srv::SCPackRateCommand>::SharedPtr sc_pack_rate_client_;
  std::unique_ptr<rclcpp::SyncParametersClient> sc_pack_rate_param_client_;
  rclcpp::Client<buoy_interfaces::srv::PCScaleCommand>::SharedPtr pc_scale_client_;
  rclcpp::Client<buoy_interfaces::srv::PCRetractCommand>::SharedPtr pc_retract_client_;
  rclcpp::Client<buoy_interfaces::srv::PCVTargMaxCommand>::SharedPtr pc_v_targ_max_client_;
  rclcpp::Client<
    buoy_interfaces::srv::PCChargeCurrLimCommand
  >::SharedPtr pc_charge_curr_lim_client_;
  rclcpp::Client<buoy_interfaces::srv::PCBattSwitchCommand>::SharedPtr pc_batt_switch_client_;
  rclcpp::Client<buoy_interfaces::srv::GainCommand>::SharedPtr gain_client_;
  rclcpp::Client<buoy_interfaces::srv::PCStdDevTargCommand>::SharedPtr pc_std_dev_targ_client_;
  rclcpp::Client<buoy_interfaces::srv::PCDrawCurrLimCommand>::SharedPtr pc_draw_curr_lim_client_;
  rclcpp::Client<buoy_interfaces::srv::PCWindCurrCommand>::SharedPtr pc_wind_curr_client_;
  rclcpp::Client<buoy_interfaces::srv::PCBiasCurrCommand>::SharedPtr pc_bias_curr_client_;
  rclcpp::Client<buoy_interfaces::srv::PCPackRateCommand>::SharedPtr pc_pack_rate_client_;
  std::unique_ptr<rclcpp::SyncParametersClient> pc_pack_rate_param_client_;
  rclcpp::Client<buoy_interfaces::srv::TFSetPosCommand>::SharedPtr tf_set_pos_client_;
  rclcpp::Client<buoy_interfaces::srv::TFSetActualPosCommand>::SharedPtr tf_set_actual_pos_client_;
  rclcpp::Client<buoy_interfaces::srv::TFSetModeCommand>::SharedPtr tf_set_mode_client_;
  rclcpp::Client<
    buoy_interfaces::srv::TFSetChargeModeCommand
  >::SharedPtr tf_set_charge_mode_client_;
  rclcpp::Client<buoy_interfaces::srv::TFSetCurrLimCommand>::SharedPtr tf_set_curr_lim_client_;
  rclcpp::Client<buoy_interfaces::srv::TFSetStateMachineCommand>::SharedPtr
    tf_set_state_machine_client_;
  rclcpp::Client<buoy_interfaces::srv::TFWatchDogCommand>::SharedPtr tf_watch_dog_client_;
  rclcpp::Client<buoy_interfaces::srv::TFResetCommand>::SharedPtr tf_reset_client_;

  // generic service callback
  template<class CallbackType, class ServiceResponseFuture>
  CallbackType default_service_response_callback()
  {
    CallbackType callback = [this](ServiceResponseFuture future)
      {
        if (future.get()->result.value == future.get()->result.OK) {
          RCLCPP_INFO(
            rclcpp::get_logger(this->get_name()),
            "Command Successful");
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger(this->get_name()),
            "Command Failed: received error code [[ %s ]]",
            pbsrv_enum2str[future.get()->result.value].c_str());
        }
      };

    return callback;
  }

private:
  template<class T>
  bool wait_for_service(T & client, const std::string & service, const size_t & _count = 1U)
  {
    // NOLINTNEXTLINE
    using namespace std::chrono_literals;
    size_t count{0U};
    while (count < _count && !client->wait_for_service(100ms)) {
      ++count;
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(this->get_name()),
          "Interrupted while waiting for %s. Exiting.",
          service.c_str());
        return false;
      }
      RCLCPP_INFO(
        rclcpp::get_logger(this->get_name()),
        "%s not available, still waiting...",
        service.c_str());
    }
    return count < _count;
  }

  // declare all subscribers
  rclcpp::Subscription<buoy_interfaces::msg::XBRecord>::SharedPtr ahrs_data_sub_, xb_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::BCRecord>::SharedPtr battery_data_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::BCRecord>::SharedPtr bc_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::SCRecord>::SharedPtr spring_data_sub_, sc_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::PCRecord>::SharedPtr power_data_sub_, pc_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::TFRecord>::SharedPtr trefoil_data_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::TFRecord>::SharedPtr tf_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::PBRecord>::SharedPtr powerbuoy_data_sub_;
};

}  // namespace buoy_api

#endif  // BUOY_API__INTERFACE_HPP_
