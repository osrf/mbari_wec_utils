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

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// Pack Rate Params
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/parameter.hpp>

// pbsrv commands
// power microcontroller
#include <buoy_interfaces/srv/pc_batt_switch_command.hpp>
#include <buoy_interfaces/srv/pc_bias_curr_command.hpp>
#include <buoy_interfaces/srv/pc_charge_curr_lim_command.hpp>
#include <buoy_interfaces/srv/pc_draw_curr_lim_command.hpp>
#include <buoy_interfaces/srv/pc_pack_rate_command.hpp>
#include <buoy_interfaces/srv/pc_retract_command.hpp>
#include <buoy_interfaces/srv/pc_scale_command.hpp>
#include <buoy_interfaces/srv/pc_std_dev_targ_command.hpp>
#include <buoy_interfaces/srv/pcv_targ_max_command.hpp>
#include <buoy_interfaces/srv/pc_wind_curr_command.hpp>
#include <buoy_interfaces/srv/gain_command.hpp>

// battery microcontroller
#include <buoy_interfaces/srv/bc_reset_command.hpp>

// spring microcontroller
#include <buoy_interfaces/srv/sc_pack_rate_command.hpp>
#include <buoy_interfaces/srv/sc_reset_command.hpp>
#include <buoy_interfaces/srv/valve_command.hpp>
#include <buoy_interfaces/srv/pump_command.hpp>
#include <buoy_interfaces/srv/bender_command.hpp>
#include <buoy_interfaces/srv/tether_command.hpp>

// trefoil microcontroller
#include <buoy_interfaces/srv/tf_reset_command.hpp>
#include <buoy_interfaces/srv/tf_set_actual_pos_command.hpp>
#include <buoy_interfaces/srv/tf_set_charge_mode_command.hpp>
#include <buoy_interfaces/srv/tf_set_curr_lim_command.hpp>
#include <buoy_interfaces/srv/tf_set_mode_command.hpp>
#include <buoy_interfaces/srv/tf_set_pos_command.hpp>
#include <buoy_interfaces/srv/tf_set_state_machine_command.hpp>
#include <buoy_interfaces/srv/tf_watch_dog_command.hpp>

// pb telemetry
#include <buoy_interfaces/msg/xb_record.hpp>  // ahrs
#include <buoy_interfaces/msg/bc_record.hpp>  // battery
#include <buoy_interfaces/msg/sc_record.hpp>  // spring
#include <buoy_interfaces/msg/pc_record.hpp>  // power
#include <buoy_interfaces/msg/tf_record.hpp>  // trefoil
#include <buoy_interfaces/msg/pb_record.hpp>  // consolidated

// Sim Only
#include <buoy_interfaces/msg/latent_data.hpp>
#include <buoy_interfaces/srv/inc_wave_height.hpp>

#include <geometry_msgs/msg/point.hpp>

namespace buoy_api
{
using std::placeholders::_1;

std::map<int8_t, std::string> pbsrv_enum2str = {{0, "OK"},
  {-1, "BAD_SOCK"},
  {-2, "BAD_OPTS"},
  {-3, "BAD_INPUT"},
  {-4, "BUSY"}};


/**
 * @brief ROS 2 Interface node for commanding and subscribing to buoy controllers and sensors.
 *
 * This template class uses the Curiously Recurring Template Pattern (CRTP) to provide a
 * compile-time polymorphic interface for creating node-based controllers. By using CRTP,
 * derived classes can override callback methods and parameter-setting behavior without incurring
 * the overhead of virtual dispatch. The derived class must pass itself as the template parameter
 * to `Interface`, enabling the base class to call into user-defined implementations.
 *
 * Provides service clients and functions to send commands to and receive telemetry from the
 * MBARI WEC controllers:
 *
 * - AHRS
 * - Power
 * - Spring
 * - Battery
 * - Trefoil
 *
 * If the user has overridden one of these callbacks in their derived class, the corresponding
 * topic subscriber will be set up and routed to their implementation. If not, that topic will
 * not be subscribed to. The relevant callbacks include:
 *
 * - ahrs_callback
 * - battery_callback
 * - spring_callback
 * - power_callback
 * - trefoil_callback
 * - powerbuoy_callback
 * - latent_callback (SIM ONLY)
 *
 * @tparam ControllerImplCRTP The concrete controller class that inherits from this interface.
 *                 It must implement any callbacks or parameter-setting routines
 *                 it needs, marked as `final`.
 *
 * ## How to Use
 *
 * 1. Include the header for `Interface`:
 *    @code{.cpp}
 *    #include <buoy_api/interface.hpp>
 *    @endcode
 *
 * 2. Forward-declare any policies or helper classes you will use:
 *    @code{.cpp}
 *    struct PBTorqueControlPolicy;  // defined by user in torque_control_policy.hpp
 *    @endcode
 *
 * 3. Define your controller class by inheriting from `buoy_api::Interface<YourClass>`
 *    and adding `friend CRTP`:
 *    @code{.cpp}
 *    class PBTorqueController final : public buoy_api::Interface<PBTorqueController>
 *    {
 *    public:
 *      explicit PBTorqueController(const std::string & node_name);
 *      ~PBTorqueController() = default;
 *
 *    private:
 *      friend CRTP;  // Enables base to access overrides.
 *      void set_params() final;
 *      void power_callback(const buoy_interfaces::msg::PCRecord & data);
 *      std::unique_ptr<PBTorqueControlPolicy> policy_;
 *    };
 *    @endcode
 *
 * 4. Implement `set_params()` to declare or update ROS2 parameters to update your policy class.
 *
 * 5. Override any telemetry callbacks to process incoming data.
 *
 * 6. Construct your controller in your `main()` function, passing in the desired node name.
 *    The base `Interface` handles common setup: parameters, services, publishers, and subscribers.
 *
 * ### Benefits of CRTP in This Context
 * - **Zero-cost abstraction**: No virtual table; callbacks are resolved at compile time.
 * - **Flexible overrides**: Only override what you use.
 * - **Simplified boilerplate**: Base class manages ROS2 setup.
 */
template<class ControllerImplCRTP>
class Interface : public rclcpp::Node
{
public:
  using CRTP = Interface;  // syntactic sugar for friend class
                           // see https://stackoverflow.com/a/58435857/9686600
  explicit Interface(const std::string & node_name)
  : Interface(node_name, false, true)
  {
  }

  /**
   * @brief Initialize the Interface node.
   *
   * @param node_name Name of the ROS2 node.
   * @param _wait_for_services If true and check_for_services, block until services are available.
   * @param _check_for_services If true, attempt to verify service availability before use.
   */
  explicit Interface(
    const std::string & node_name,
    const bool _wait_for_services,
    const bool _check_for_services)
  : Node(node_name)
  {
    using rclcpp::CallbackGroupType;
    cb_sub_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
    cb_cli_ = this->create_callback_group(CallbackGroupType::MutuallyExclusive);

    // Default QoS
    auto service_qos = rclcpp::QoS(rclcpp::ServicesQoS());

    pc_pack_rate_param_client_ =
      std::make_unique<rclcpp::SyncParametersClient>(
        std::shared_ptr<rclcpp::Node>(
          static_cast<ControllerImplCRTP *>(this),
        [](rclcpp::Node *) {}
      ),
        "/power_controller"
      );
    pc_pack_rate_client_ =
      this->create_client<buoy_interfaces::srv::PCPackRateCommand>(
        "/pc_pack_rate_command",
        service_qos,
        cb_cli_
      );
    pc_wind_curr_client_ =
      this->create_client<buoy_interfaces::srv::PCWindCurrCommand>(
        "/pc_wind_curr_command",
        service_qos,
        cb_cli_
      );
    bender_client_ =
      this->create_client<buoy_interfaces::srv::BenderCommand>(
        "/bender_command",
        service_qos,
        cb_cli_
      );
    bc_reset_client_ =
      this->create_client<buoy_interfaces::srv::BCResetCommand>(
        "/bc_reset_command",
        service_qos,
        cb_cli_
      );
    pump_client_ =
      this->create_client<buoy_interfaces::srv::PumpCommand>(
        "/pump_command",
        service_qos,
        cb_cli_
      );
    valve_client_ =
      this->create_client<buoy_interfaces::srv::ValveCommand>(
        "/valve_command",
        service_qos,
        cb_cli_
      );
    tether_client_ =
      this->create_client<buoy_interfaces::srv::TetherCommand>(
        "/tether_command",
        service_qos,
        cb_cli_
      );
    sc_reset_client_ =
      this->create_client<buoy_interfaces::srv::SCResetCommand>(
        "/sc_reset_command",
        service_qos,
        cb_cli_
      );
    sc_pack_rate_param_client_ =
      std::make_unique<rclcpp::SyncParametersClient>(
        std::shared_ptr<rclcpp::Node>(
          static_cast<ControllerImplCRTP *>(this),
        [](rclcpp::Node *) {}
      ),
        "/spring_controller"
      );
    sc_pack_rate_client_ =
      this->create_client<buoy_interfaces::srv::SCPackRateCommand>(
        "/sc_pack_rate_command",
        service_qos,
        cb_cli_
      );
    pc_scale_client_ =
      this->create_client<buoy_interfaces::srv::PCScaleCommand>(
        "/pc_scale_command",
        service_qos,
        cb_cli_
      );
    pc_retract_client_ =
      this->create_client<buoy_interfaces::srv::PCRetractCommand>(
        "/pc_retract_command",
        service_qos,
        cb_cli_
      );
    pc_v_targ_max_client_ =
      this->create_client<buoy_interfaces::srv::PCVTargMaxCommand>(
        "/pc_v_targ_max_command",
        service_qos,
        cb_cli_
      );
    pc_charge_curr_lim_client_ =
      this->create_client<buoy_interfaces::srv::PCChargeCurrLimCommand>(
        "/pc_charge_curr_lim_command",
        service_qos,
        cb_cli_
      );
    pc_batt_switch_client_ =
      this->create_client<buoy_interfaces::srv::PCBattSwitchCommand>(
        "/pc_batt_switch_command",
        service_qos,
        cb_cli_
      );
    gain_client_ =
      this->create_client<buoy_interfaces::srv::GainCommand>(
        "/gain_command",
        service_qos,
        cb_cli_
      );
    pc_std_dev_targ_client_ =
      this->create_client<buoy_interfaces::srv::PCStdDevTargCommand>(
        "/pc_std_dev_targ_command",
        service_qos,
        cb_cli_
      );
    pc_draw_curr_lim_client_ =
      this->create_client<buoy_interfaces::srv::PCDrawCurrLimCommand>(
        "/pc_draw_curr_lim_command",
        service_qos,
        cb_cli_
      );
    pc_bias_curr_client_ =
      this->create_client<buoy_interfaces::srv::PCBiasCurrCommand>(
        "/pc_bias_curr_command",
        service_qos,
        cb_cli_
      );
    tf_set_pos_client_ =
      this->create_client<buoy_interfaces::srv::TFSetPosCommand>(
        "/tf_set_pos_command",
        service_qos,
        cb_cli_
      );
    tf_set_actual_pos_client_ =
      this->create_client<buoy_interfaces::srv::TFSetActualPosCommand>(
        "/tf_set_actual_pos_command",
        service_qos,
        cb_cli_
      );
    tf_set_mode_client_ =
      this->create_client<buoy_interfaces::srv::TFSetModeCommand>(
        "/tf_set_mode_command",
        service_qos,
        cb_cli_
      );
    tf_set_charge_mode_client_ =
      this->create_client<buoy_interfaces::srv::TFSetChargeModeCommand>(
        "/tf_set_charge_mode_command",
        service_qos,
        cb_cli_
      );
    tf_set_curr_lim_client_ =
      this->create_client<buoy_interfaces::srv::TFSetCurrLimCommand>(
        "/tf_set_curr_lim_command",
        service_qos,
        cb_cli_
      );
    tf_set_state_machine_client_ =
      this->create_client<buoy_interfaces::srv::TFSetStateMachineCommand>(
        "/tf_set_state_machine_command",
        service_qos,
        cb_cli_
      );
    tf_watchdog_client_ =
      this->create_client<buoy_interfaces::srv::TFWatchDogCommand>(
        "/tf_watchdog_command",
        service_qos,
        cb_cli_
      );
    tf_reset_client_ =
      this->create_client<buoy_interfaces::srv::TFResetCommand>(
        "/tf_reset_command",
        service_qos,
        cb_cli_
      );
    inc_wave_height_client_ =
      this->create_client<buoy_interfaces::srv::IncWaveHeight>(
        "/inc_wave_height",
        service_qos,
        cb_cli_
      );

    setup_subscribers();
    if (_check_for_services) {
      bool found = false;
      do {
        found = wait_for_services();
      } while (rclcpp::ok() && !found && _wait_for_services);
      RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Found all required services.");
    }
  }

  void spin()
  {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(static_cast<ControllerImplCRTP *>(this)->shared_from_this());
    executor.spin();
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
    found &= wait_for_service(tf_watchdog_client_, "/tf_watchdog_command");
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
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to XBRecord on '/ahrs_data' and '/xb_record'");
      ahrs_data_sub_ = this->create_subscription<buoy_interfaces::msg::XBRecord>(
        "/ahrs_data", 1,
        std::bind(
          &ControllerImplCRTP::ahrs_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
      xb_record_sub_ = this->create_subscription<buoy_interfaces::msg::XBRecord>(
        "/xb_record", 1,
        std::bind(
          &ControllerImplCRTP::ahrs_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
    }

    if (&ControllerImplCRTP::battery_callback == &Interface::battery_callback) {
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to BCRecord on '/battery_data' and '/bc_record'");
      battery_data_sub_ = this->create_subscription<buoy_interfaces::msg::BCRecord>(
        "/battery_data", 1,
        std::bind(
          &ControllerImplCRTP::battery_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
      bc_record_sub_ = this->create_subscription<buoy_interfaces::msg::BCRecord>(
        "/bc_record", 1,
        std::bind(
          &ControllerImplCRTP::battery_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
    }

    if (&ControllerImplCRTP::spring_callback == &Interface::spring_callback) {
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to SCRecord on '/spring_data' and '/sc_record'");
      spring_data_sub_ = this->create_subscription<buoy_interfaces::msg::SCRecord>(
        "/spring_data", 1,
        std::bind(
          &ControllerImplCRTP::spring_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
      sc_record_sub_ = this->create_subscription<buoy_interfaces::msg::SCRecord>(
        "/sc_record", 1,
        std::bind(
          &ControllerImplCRTP::spring_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
    }

    if (&ControllerImplCRTP::power_callback == &Interface::power_callback) {
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to PCRecord on '/power_data' and '/pc_record'");
      power_data_sub_ = this->create_subscription<buoy_interfaces::msg::PCRecord>(
        "/power_data", 1,
        std::bind(
          &ControllerImplCRTP::power_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
      pc_record_sub_ = this->create_subscription<buoy_interfaces::msg::PCRecord>(
        "/pc_record", 1,
        std::bind(
          &ControllerImplCRTP::power_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
    }

    if (&ControllerImplCRTP::trefoil_callback == &Interface::trefoil_callback) {
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to TFRecord on '/trefoil_data' and '/tf_record'");
      trefoil_data_sub_ = this->create_subscription<buoy_interfaces::msg::TFRecord>(
        "/trefoil_data", 1,
        std::bind(
          &ControllerImplCRTP::trefoil_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
      tf_record_sub_ = this->create_subscription<buoy_interfaces::msg::TFRecord>(
        "/tf_record", 1,
        std::bind(
          &ControllerImplCRTP::trefoil_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
      );
    }

    if (&ControllerImplCRTP::powerbuoy_callback == &Interface::powerbuoy_callback) {
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to PBRecord on '/powerbuoy_data'");
      powerbuoy_data_sub_ =
        this->create_subscription<buoy_interfaces::msg::PBRecord>(
        "/powerbuoy_data", 1,
        std::bind(
          &ControllerImplCRTP::powerbuoy_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
        );
    }

    if (&ControllerImplCRTP::latent_callback == &Interface::latent_callback) {
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_sub_;
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(this->get_name()),
        "Subscribing to LatentData on '/latent_data'");
      latent_data_sub_ =
        this->create_subscription<buoy_interfaces::msg::LatentData>(
        "/latent_data", 1,
        std::bind(
          &ControllerImplCRTP::latent_callback,
          static_cast<ControllerImplCRTP *>(this), _1),
        sub_opts
        );
    }
  }

  /**
   * @brief Enable/Disable using sim time in Node clock from /clock.
   *
   * @param enable True to use /clock, False to use system time.
   */
  void use_sim_time(bool enable = true)
  {
    this->set_parameter(
      rclcpp::Parameter(
        "use_sim_time",
        enable));
  }

  /**
   * @brief Set publish rate of PC Microcontroller telemetry via parameter server.
   *
   * @param rate_hz Desired publish rate in Hz.
   */
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

  /**
   * @brief Set publish rate of SC Microcontroller telemetry via parameter server.
   *
   * @param rate_hz Desired publish rate in Hz.
   */
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
  using IncWaveHeightServiceResponseFuture =
    rclcpp::Client<buoy_interfaces::srv::IncWaveHeight>::SharedFuture;

  /**
   * @brief Set publish rate of PC Microcontroller telemetry.
   *
   * @param rate_hz Desired publish rate in Hz.
   */
  PCPackRateServiceResponseFuture set_pc_pack_rate(
    const uint8_t & rate_hz = 50,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCPackRateCommand::Request>();
    request->rate_hz = rate_hz;

    PCPackRateServiceResponseFuture pc_pack_rate_response_future;

    if (use_callback) {
      PCPackRateServiceCallback pc_pack_rate_callback =
        default_service_response_callback<PCPackRateServiceCallback,
          PCPackRateServiceResponseFuture>();

      // NOTE: Move semantics destroys local pc_pack_rate_callback object
      auto shared_future_and_request_id = pc_pack_rate_client_->async_send_request(
        request,
        pc_pack_rate_callback
      );

      pc_pack_rate_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = pc_pack_rate_client_->async_send_request(request);
      pc_pack_rate_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          pc_pack_rate_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for PC Pack Rate Command"
          );
        }
      } else {
        pc_pack_rate_response_future.wait();
      }
    }

    return pc_pack_rate_response_future;
  }

  /**
   * @brief Set publish rate of SC Microcontroller telemetry.
   *
   * @param rate_hz Desired publish rate in Hz.
   */
  SCPackRateServiceResponseFuture set_sc_pack_rate(
    const uint8_t & rate_hz = 50,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::SCPackRateCommand::Request>();
    request->rate_hz = rate_hz;

    SCPackRateServiceResponseFuture sc_pack_rate_response_future;

    if (use_callback) {
      SCPackRateServiceCallback sc_pack_rate_callback =
        default_service_response_callback<SCPackRateServiceCallback,
          SCPackRateServiceResponseFuture>();

      auto shared_future_and_request_id = sc_pack_rate_client_->async_send_request(
        request,
        sc_pack_rate_callback
      );

      sc_pack_rate_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = sc_pack_rate_client_->async_send_request(request);
      sc_pack_rate_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          sc_pack_rate_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for SC Pack Rate Command"
          );
        }
      } else {
        sc_pack_rate_response_future.wait();
      }
    }

    return sc_pack_rate_response_future;
  }

  /**
   * @brief Turn valve on for a duration to lower mean piston position.
   *
   * @param duration_sec Valve on duration in seconds.
   * @return A future containing the service response.
   */
  ValveServiceResponseFuture send_valve_command(
    const uint16_t & duration_sec,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::ValveCommand::Request>();
    request->duration_sec = duration_sec;

    ValveServiceResponseFuture valve_response_future;

    if (use_callback) {
      ValveServiceCallback valve_callback =
        default_service_response_callback<ValveServiceCallback,
          ValveServiceResponseFuture>();

      auto shared_future_and_request_id = valve_client_->async_send_request(
        request,
        valve_callback
      );

      valve_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = valve_client_->async_send_request(request);
      valve_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          valve_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for Valve Command"
          );
        }
      } else {
        valve_response_future.wait();
      }
    }

    return valve_response_future;
  }

  /**
   * @brief Turn pump on for a duration to raise mean piston position.
   *
   * @param duration_mins Pump on duration in minutes.
   * @return A future containing the service response.
   */
  PumpServiceResponseFuture send_pump_command(
    const float & duration_mins,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::PumpCommand::Request>();
    request->duration_mins = duration_mins;

    PumpServiceResponseFuture pump_response_future;

    if (use_callback) {
      PumpServiceCallback pump_callback =
        default_service_response_callback<PumpServiceCallback,
          PumpServiceResponseFuture>();

      auto shared_future_and_request_id = pump_client_->async_send_request(
        request,
        pump_callback
      );

      pump_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = pump_client_->async_send_request(request);
      pump_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          pump_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for Pump Command"
          );
        }
      } else {
        pump_response_future.wait();
      }
    }

    return pump_response_future;
  }

  /**
   * @brief Set winding current setpoint to control piston damping.
   *
   * @param wind_curr Wind current setpoint in Amps.
   * @return A future containing the service response.
   */
  PCWindCurrServiceResponseFuture send_pc_wind_curr_command(
    const float & wind_curr,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCWindCurrCommand::Request>();
    request->wind_curr = wind_curr;

    PCWindCurrServiceResponseFuture pc_wind_curr_response_future;

    if (use_callback) {
      PCWindCurrServiceCallback pc_wind_curr_callback =
        default_service_response_callback<PCWindCurrServiceCallback,
          PCWindCurrServiceResponseFuture>();

      // NOTE: Move semantics destroys local
      // pc_wind_curr_callback object
      auto shared_future_and_request_id = pc_wind_curr_client_->async_send_request(
        request,
        pc_wind_curr_callback
      );

      pc_wind_curr_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = pc_wind_curr_client_->async_send_request(request);

      pc_wind_curr_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          pc_wind_curr_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out wiating for PC Wind Current Command"
          );
        }
      } else {
        pc_wind_curr_response_future.wait();
      }
    }

    return pc_wind_curr_response_future;
  }

  /**
   * @brief Set bias current setpoint to control piston damping offset.
   *
   * A high bias in either direction will move the piston back and forth.
   *
   * @param bias_curr Bias current setpoint in Amps.
   * @return A future containing the service response.
   */
  PCBiasCurrServiceResponseFuture send_pc_bias_curr_command(
    const float & bias_curr,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCBiasCurrCommand::Request>();
    request->bias_curr = bias_curr;

    PCBiasCurrServiceResponseFuture pc_bias_curr_response_future;

    if (use_callback) {
      PCBiasCurrServiceCallback pc_bias_curr_callback =
        default_service_response_callback<PCBiasCurrServiceCallback,
          PCBiasCurrServiceResponseFuture>();

      auto shared_future_and_request_id = pc_bias_curr_client_->async_send_request(
        request,
        pc_bias_curr_callback
      );

      pc_bias_curr_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = pc_bias_curr_client_->async_send_request(request);
      pc_bias_curr_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          pc_bias_curr_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for PC Bias Current Command"
          );
        }
      } else {
        pc_bias_curr_response_future.wait();
      }
    }

    return pc_bias_curr_response_future;
  }

  /**
   * @brief Set damping gain.
   *
   * @param scale Damping gain.
   * @return A future containing the service response.
   */
  PCScaleServiceResponseFuture send_pc_scale_command(
    const float & scale,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCScaleCommand::Request>();
    request->scale = scale;

    PCScaleServiceResponseFuture pc_scale_response_future;

    if (use_callback) {
      PCScaleServiceCallback pc_scale_callback =
        default_service_response_callback<PCScaleServiceCallback,
          PCScaleServiceResponseFuture>();

      auto shared_future_and_request_id = pc_scale_client_->async_send_request(
        request,
        pc_scale_callback
      );

      pc_scale_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = pc_scale_client_->async_send_request(request);
      pc_scale_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          pc_scale_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for PC Scale Command"
          );
        }
      } else {
        pc_scale_response_future.wait();
      }
    }

    return pc_scale_response_future;
  }

  /**
   * @brief Set additional damping gain in the piston retract direction.
   *
   * @param retract Additional damping gain for retraction.
   * @return A future containing the service response.
   */
  PCRetractServiceResponseFuture send_pc_retract_command(
    const float & retract,
    bool blocking = false,
    float timeout = 0.0,
    bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::PCRetractCommand::Request>();
    request->retract = retract;

    PCRetractServiceResponseFuture pc_retract_response_future;

    if (use_callback) {
      PCRetractServiceCallback pc_retract_callback =
        default_service_response_callback<PCRetractServiceCallback,
          PCRetractServiceResponseFuture>();

      auto shared_future_and_request_id = pc_retract_client_->async_send_request(
        request,
        pc_retract_callback
      );

      pc_retract_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = pc_retract_client_->async_send_request(request);
      pc_retract_response_future = future_and_request_id.future.share();
    }

    if (blocking) {
      if (timeout > 0.0) {
        std::chrono::steady_clock::time_point timeout_ =
          std::chrono::steady_clock::now() +
          std::chrono::microseconds(static_cast<int>(timeout * 1e6));

        if (std::future_status::ready !=
          pc_retract_response_future.wait_until(timeout_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(this->get_name()),
            "Timed out waiting for PC Retract Command"
          );
        }
      } else {
        pc_retract_response_future.wait();
      }
    }

    return pc_retract_response_future;
  }

  /**
   * @brief Set additional damping gain in the piston retract direction.
   *
   * @param retract Additional damping gain for retraction.
   * @return A future containing the service response.
   */
  buoy_interfaces::srv::IncWaveHeight::Response::SharedPtr get_inc_wave_height(
    const float x,
    const float y,
    const float t,
    const bool use_buoy_origin = false,
    const bool use_relative_time = false,
    const float timeout = 2.0,
    const bool use_callback = false
  )
  {
    std::vector<float> x_, y_, t_;
    x_.push_back(x);
    y_.push_back(y);
    t_.push_back(t);

    return get_inc_wave_height(
      x_, y_, t_,
      use_buoy_origin,
      use_relative_time,
      timeout, use_callback
    );
  }

  buoy_interfaces::srv::IncWaveHeight::Response::SharedPtr get_inc_wave_height(
    const std::vector<float> x,
    const std::vector<float> y,
    const std::vector<float> t,
    const bool use_buoy_origin = false,
    const bool use_relative_time = false,
    const float timeout = 2.0,
    const bool use_callback = false
  )
  {
    auto request = std::make_shared<buoy_interfaces::srv::IncWaveHeight::Request>();
    request->use_buoy_origin = use_buoy_origin;
    request->use_relative_time = use_relative_time;

    for (std::size_t idx = 0U; idx < x.size(); ++idx) {
      geometry_msgs::msg::Point pt;
      pt.x = x[idx];
      pt.y = y[idx];
      request->points.push_back(pt);

      if (use_relative_time) {
        request->relative_time.push_back(t[idx]);
      } else {
        request->absolute_time.push_back(t[idx]);
      }
    }

    IncWaveHeightServiceResponseFuture inc_wave_height_response_future;

    if (use_callback) {
      IncWaveHeightServiceCallback inc_wave_height_callback =
        default_service_response_callback<IncWaveHeightServiceCallback,
          IncWaveHeightServiceResponseFuture>();

      auto shared_future_and_request_id = inc_wave_height_client_->async_send_request(
        request,
        inc_wave_height_callback
      );

      inc_wave_height_response_future = std::move(shared_future_and_request_id.future);
    } else {
      auto future_and_request_id = inc_wave_height_client_->async_send_request(request);
      inc_wave_height_response_future = future_and_request_id.future.share();
    }

    if (timeout > 0.0) {
      std::chrono::steady_clock::time_point timeout_ =
        std::chrono::steady_clock::now() +
        std::chrono::microseconds(static_cast<int>(timeout * 1e6));

      if (std::future_status::ready !=
        inc_wave_height_response_future.wait_until(timeout_))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger(this->get_name()),
          "Timed out waiting for IncWaveHeight data"
        );
      }
    } else {
      inc_wave_height_response_future.wait();
    }

    return inc_wave_height_response_future.get();
  }

protected:
  virtual ~Interface() = default;

  // set_params and callbacks optionally defined by user

  /**
   * @brief Set user-defined Node parameters (e.g., custom controller gains).
   */
  virtual void set_params() {}

  /**
   * @brief Override this function to subscribe to /ahrs_data to receive XBRecord telemetry.
   *
   * @param data Incoming XBRecord.
   */
  void ahrs_callback(const buoy_interfaces::msg::XBRecord &) {}

  /**
   * @brief Override this function to subscribe to /battery_data to receive BCRecord telemetry.
   *
   * @param data Incoming BCRecord.
   */
  void battery_callback(const buoy_interfaces::msg::BCRecord &) {}

  /**
   * @brief Override this function to subscribe to /spring_data to receive SCRecord telemetry.
   *
   * @param data Incoming SCRecord.
   */
  void spring_callback(const buoy_interfaces::msg::SCRecord &) {}

  /**
   * @brief Override this function to subscribe to /power_data to receive PCRecord telemetry.
   *
   * @param data Incoming PCRecord.
   */
  void power_callback(const buoy_interfaces::msg::PCRecord &) {}

  /**
   * @brief Override this function to subscribe to /trefoil_data to receive TFRecord telemetry.
   *
   * @param data Incoming TFRecord.
   */
  void trefoil_callback(const buoy_interfaces::msg::TFRecord &) {}

  /**
   * @brief Override this function to subscribe to /powerbuoy_data to receive PBRecord telemetry.
   *
   * @param data Incoming PBRecord containing a slice of all microcontroller telemetry data.
   */
  void powerbuoy_callback(const buoy_interfaces::msg::PBRecord &) {}

  /**
   * @brief Override this function to subscribe to /latent_data to receive LatentData sim-only data.
   *
   * @param data Incoming LatentData containing sim-only values e.g. losses, waves, etc.
   */
  void latent_callback(const buoy_interfaces::msg::LatentData &) {}

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
  using IncWaveHeightServiceCallback =
    rclcpp::Client<buoy_interfaces::srv::IncWaveHeight>::CallbackType;

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
   * IncWaveHeightServiceCallback inc_wave_height_callback =
   *   default_service_response_callback<IncWaveHeightServiceCallback,
   *     IncWaveHeightServiceResponseFuture>();
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
  rclcpp::Client<buoy_interfaces::srv::TFWatchDogCommand>::SharedPtr tf_watchdog_client_;
  rclcpp::Client<buoy_interfaces::srv::TFResetCommand>::SharedPtr tf_reset_client_;
  rclcpp::Client<buoy_interfaces::srv::IncWaveHeight>::SharedPtr inc_wave_height_client_;

  // generic service callback
  template<class CallbackType, class ServiceResponseFuture>
  CallbackType default_service_response_callback()
  {
    CallbackType callback = [this](ServiceResponseFuture future)
      {
        if (future.get()->result.value == future.get()->result.OK) {
          RCLCPP_DEBUG(
            rclcpp::get_logger(this->get_name()),
            "Command Successful");
        } else {
          RCLCPP_ERROR(
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
        "%s not available",
        service.c_str());
    }
    return count < _count;
  }

  rclcpp::CallbackGroup::SharedPtr cb_sub_;
  rclcpp::CallbackGroup::SharedPtr cb_cli_;

  // declare all subscribers
  rclcpp::Subscription<buoy_interfaces::msg::XBRecord>::SharedPtr ahrs_data_sub_, xb_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::BCRecord>::SharedPtr battery_data_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::BCRecord>::SharedPtr bc_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::SCRecord>::SharedPtr spring_data_sub_, sc_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::PCRecord>::SharedPtr power_data_sub_, pc_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::TFRecord>::SharedPtr trefoil_data_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::TFRecord>::SharedPtr tf_record_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::PBRecord>::SharedPtr powerbuoy_data_sub_;
  rclcpp::Subscription<buoy_interfaces::msg::LatentData>::SharedPtr latent_data_sub_;
};

}  // namespace buoy_api

#endif  // BUOY_API__INTERFACE_HPP_
