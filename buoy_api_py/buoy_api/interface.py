# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import concurrent
from functools import partial
import threading

import numpy as np

# pbsrv commands
# power microcontroller
from buoy_interfaces.srv import GainCommand  # noqa
from buoy_interfaces.srv import PCBattSwitchCommand  # noqa
from buoy_interfaces.srv import PCBiasCurrCommand  # noqa
from buoy_interfaces.srv import PCChargeCurrLimCommand  # noqa
from buoy_interfaces.srv import PCDrawCurrLimCommand  # noqa
from buoy_interfaces.srv import PCPackRateCommand  # noqa
from buoy_interfaces.srv import PCRetractCommand  # noqa
from buoy_interfaces.srv import PCScaleCommand  # noqa
from buoy_interfaces.srv import PCStdDevTargCommand  # noqa
from buoy_interfaces.srv import PCVTargMaxCommand  # noqa
from buoy_interfaces.srv import PCWindCurrCommand  # noqa

# battery microcontroller
from buoy_interfaces.srv import BCResetCommand  # noqa

# spring microcontroller
from buoy_interfaces.srv import BenderCommand  # noqa
from buoy_interfaces.srv import PumpCommand  # noqa
from buoy_interfaces.srv import SCResetCommand  # noqa
from buoy_interfaces.srv import SCPackRateCommand  # noqa
from buoy_interfaces.srv import TetherCommand  # noqa
from buoy_interfaces.srv import ValveCommand  # noqa

# trefoil microcontroller
from buoy_interfaces.srv import TFResetCommand  # noqa
from buoy_interfaces.srv import TFSetActualPosCommand  # noqa
from buoy_interfaces.srv import TFSetChargeModeCommand  # noqa
from buoy_interfaces.srv import TFSetCurrLimCommand  # noqa
from buoy_interfaces.srv import TFSetModeCommand  # noqa
from buoy_interfaces.srv import TFSetPosCommand  # noqa
from buoy_interfaces.srv import TFSetStateMachineCommand  # noqa
from buoy_interfaces.srv import TFWatchDogCommand  # noqa


# pb telemetry
from buoy_interfaces.msg import BCRecord  # battery  # noqa
from buoy_interfaces.msg import PBRecord  # consolidated  # noqa
from buoy_interfaces.msg import PCRecord  # power  # noqa
from buoy_interfaces.msg import SCRecord  # spring  # noqa
from buoy_interfaces.msg import TFRecord  # trefoil  # noqa
from buoy_interfaces.msg import XBRecord  # ahrs  # noqa

# sim only data
from buoy_interfaces.msg import LatentData  # noqa
from buoy_interfaces.srv import IncWaveHeight  # noqa

from geometry_msgs.msg import Point

# Pack Rate Params
from rclpy.parameter import Parameter  # noqa
from rcl_interfaces.srv import SetParameters  # noqa

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


pbsrv_enum2str = {0: 'OK',
                  -1: 'BAD_SOCK',
                  -2: 'BAD_OPTS',
                  -3: 'BAD_INPUT',
                  -4: 'BUSY'}


class Interface(Node):
    """
    ROS2 Interface node for commanding and subscribing to buoy controllers and sensors.

    Provides service clients and functions to send commands to and recieve telemetry from the
    MBARI WEC controllers:
        - AHRS
        - Power
        - Spring
        - Battery
        - Trefoil

    Also includes interfaces to sim-only latent data and incident wave heights.

    If user has overridden one of these callbacks in their user-derived class, this will
    subscribe to the correct topic and use their callback implementation. If user did not define
    one, a subscriber for that topic will not be set up:
        - self.ahrs_callback
        - self.battery_callback
        - self.spring_callback
        - self.power_callback
        - self.trefoil_callback
        - self.powerbuoy_callback
        - self.latent_callback (sim only)
    """

    def __init__(self, node_name, wait_for_services=False, check_for_services=True, **kwargs):
        """
        Initialize the Interface node.

        :param str node_name: name of the ROS2 node
        :param bool check_for_services: if True, attempt to verify service availability before use
        :param bool wait_for_services: if True and if check_for_services, block until all services
                                       are available
        :param kwargs: additional keyword arguments forwarded to ROS 2 Node
        """
        super().__init__(node_name, **kwargs)

        self.cb_sub = MutuallyExclusiveCallbackGroup()
        self.cb_cli = MutuallyExclusiveCallbackGroup()

        self.pc_pack_rate_param_client_ = self.create_client(
            SetParameters,
            '/power_controller/set_parameters',
            callback_group=self.cb_cli,
        )
        self.pc_pack_rate_client_ = self.create_client(
            PCPackRateCommand,
            '/pc_pack_rate_command',
            callback_group=self.cb_cli,
        )
        self.pc_wind_curr_client_ = self.create_client(
            PCWindCurrCommand,
            '/pc_wind_curr_command',
            callback_group=self.cb_cli,
        )
        self.bender_client_ = self.create_client(
            BenderCommand,
            '/bender_command',
            callback_group=self.cb_cli,
        )
        self.bc_reset_client_ = self.create_client(
            BCResetCommand,
            '/bc_reset_command',
            callback_group=self.cb_cli,
        )
        self.pump_client_ = self.create_client(
            PumpCommand,
            '/pump_command',
            callback_group=self.cb_cli,
        )
        self.valve_client_ = self.create_client(
            ValveCommand,
            '/valve_command',
            callback_group=self.cb_cli,
        )
        self.tether_client_ = self.create_client(
            TetherCommand,
            '/tether_command',
            callback_group=self.cb_cli,
        )
        self.sc_reset_client_ = self.create_client(
            SCResetCommand,
            '/sc_reset_command',
            callback_group=self.cb_cli,
        )
        self.sc_pack_rate_param_client_ = self.create_client(
            SetParameters,
            '/spring_controller/set_parameters',
            callback_group=self.cb_cli,
        )
        self.sc_pack_rate_client_ = self.create_client(
            SCPackRateCommand,
            '/sc_pack_rate_command',
            callback_group=self.cb_cli,
        )
        self.pc_scale_client_ = self.create_client(
            PCScaleCommand,
            '/pc_scale_command',
            callback_group=self.cb_cli,
        )
        self.pc_retract_client_ = self.create_client(
            PCRetractCommand,
            '/pc_retract_command',
            callback_group=self.cb_cli,
        )
        self.pc_v_targ_max_client_ = self.create_client(
            PCVTargMaxCommand,
            '/pc_v_targ_max_command',
            callback_group=self.cb_cli,
        )
        self.pc_charge_curr_lim_client_ = self.create_client(
            PCChargeCurrLimCommand,
            '/pc_charge_curr_lim_command',
            callback_group=self.cb_cli,
        )
        self.pc_batt_switch_client_ = self.create_client(
            PCBattSwitchCommand,
            '/pc_batt_switch_command',
            callback_group=self.cb_cli,
        )
        self.gain_client_ = self.create_client(
            GainCommand,
            '/gain_command',
            callback_group=self.cb_cli,
        )
        self.pc_std_dev_targ_client_ = self.create_client(
            PCStdDevTargCommand,
            '/pc_std_dev_targ_command',
            callback_group=self.cb_cli,
        )
        self.pc_draw_curr_lim_client_ = self.create_client(
            PCDrawCurrLimCommand,
            '/pc_draw_curr_lim_command',
            callback_group=self.cb_cli,
        )
        self.pc_bias_curr_client_ = self.create_client(
            PCBiasCurrCommand,
            '/pc_bias_curr_command',
            callback_group=self.cb_cli,
        )
        self.tf_set_pos_client_ = self.create_client(
            TFSetPosCommand,
            '/tf_set_pos_command',
            callback_group=self.cb_cli,
        )
        self.tf_set_actual_pos_client_ = self.create_client(
            TFSetActualPosCommand,
            '/tf_set_actual_pos_command',
            callback_group=self.cb_cli,
        )
        self.tf_set_mode_client_ = self.create_client(
            TFSetModeCommand,
            '/tf_set_mode_command',
            callback_group=self.cb_cli,
        )
        self.tf_set_charge_mode_client_ = self.create_client(
            TFSetChargeModeCommand,
            '/tf_set_charge_mode_command',
            callback_group=self.cb_cli,
        )
        self.tf_set_curr_lim_client_ = self.create_client(
            TFSetCurrLimCommand,
            '/tf_set_curr_lim_command',
            callback_group=self.cb_cli,
        )
        self.tf_set_state_machine_client_ = self.create_client(
            TFSetStateMachineCommand,
            '/tf_set_state_machine_command',
            callback_group=self.cb_cli,
        )
        self.tf_watchdog_client_ = self.create_client(
            TFWatchDogCommand,
            '/tf_watchdog_command',
            callback_group=self.cb_cli,
        )
        self.tf_reset_client_ = self.create_client(
            TFResetCommand,
            '/tf_reset_command',
            callback_group=self.cb_cli,
        )
        self.iwh_client_ = self.create_client(
            IncWaveHeight,
            '/inc_wave_height',
            callback_group=self.cb_cli,
        )

        self.pc_pack_rate_param_future_ = None
        self.pc_pack_rate_future_ = None
        self.pc_wind_curr_future_ = None
        self.bender_future_ = None
        self.bc_reset_future_ = None
        self.pump_future_ = None
        self.valve_future_ = None
        self.tether_future_ = None
        self.sc_reset_future_ = None
        self.sc_pack_rate_param_future_ = None
        self.sc_pack_rate_future_ = None
        self.pc_scale_future_ = None
        self.pc_retract_future_ = None
        self.pc_v_targ_max_future_ = None
        self.pc_charge_curr_lim_future_ = None
        self.pc_batt_switch_future_ = None
        self.gain_future_ = None
        self.pc_std_dev_targ_future_ = None
        self.pc_draw_curr_lim_future_ = None
        self.pc_bias_curr_future_ = None
        self.tf_set_pos_future_ = None
        self.tf_set_actual_pos_future_ = None
        self.tf_set_mode_future_ = None
        self.tf_set_charge_mode_future_ = None
        self.tf_set_curr_lim_future_ = None
        self.tf_set_state_machine_future_ = None
        self.tf_watchdog_future_ = None
        self.tf_reset_future_ = None
        self.iwh_future_ = None

        self.setup_subscribers()
        if check_for_services:
            found = self.wait_for_services()
            if not found and wait_for_services:
                while rclpy.ok() and not self.wait_for_services():
                    pass
            self.get_logger().info('Found all required services.')

    def spin(self):
        """
        Set up a `MultiThreadedExecutor` and spin the node (blocking).

        If you need non-blocking control over program flow, you may skip calling this function, but
        a `MultiThreadedExecutor` is required for this node. You may call non-blocking spin
        functions of a `MultiThreadedExecutor` in your own loop.
        """
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

    def wait_for_services(self):
        # TODO(andermi)
        # physical needs to switch from service to param for pack rates
        # sim should add all services (even unused with noop)
        found_pc_param = self.wait_for_service(self.pc_pack_rate_param_client_,
                                               '/power_controller/set_parameters')
        found_pc_packrate = self.wait_for_service(self.pc_pack_rate_client_,
                                                  '/pc_pack_rate_command')
        found = found_pc_param or found_pc_packrate
        found &= self.wait_for_service(self.pc_wind_curr_client_, '/pc_wind_curr_command')
        found &= self.wait_for_service(self.bender_client_, '/bender_command')
        found &= self.wait_for_service(self.bc_reset_client_, '/bc_reset_command')
        found &= self.wait_for_service(self.pump_client_, '/pump_command')
        found &= self.wait_for_service(self.valve_client_, '/valve_command')
        found &= self.wait_for_service(self.tether_client_, '/tether_command')
        found &= self.wait_for_service(self.sc_reset_client_, '/sc_reset_command')
        found_sc_param = self.wait_for_service(self.sc_pack_rate_param_client_,
                                               '/spring_controller/set_parameters')
        found_sc_packrate = self.wait_for_service(self.sc_pack_rate_client_,
                                                  '/sc_pack_rate_command')
        found &= found_sc_param or found_sc_packrate
        found &= self.wait_for_service(self.pc_scale_client_, '/pc_scale_command')
        found &= self.wait_for_service(self.pc_retract_client_, '/pc_retract_command')
        found &= self.wait_for_service(self.pc_v_targ_max_client_, '/pc_v_targ_max_command')
        found &= self.wait_for_service(self.pc_charge_curr_lim_client_,
                                       '/pc_charge_curr_lim_command')
        found &= self.wait_for_service(self.pc_batt_switch_client_, '/pc_batt_switch_command')
        found &= self.wait_for_service(self.gain_client_, '/gain_command')
        found &= self.wait_for_service(self.pc_std_dev_targ_client_, '/pc_std_dev_targ_command')
        found &= self.wait_for_service(self.pc_draw_curr_lim_client_,
                                       '/pc_draw_curr_lim_command')
        found &= self.wait_for_service(self.pc_bias_curr_client_, '/pc_bias_curr_command')
        found &= self.wait_for_service(self.tf_set_pos_client_, '/tf_set_pos_command')
        found &= self.wait_for_service(self.tf_set_actual_pos_client_,
                                       '/tf_set_actual_pos_command')
        found &= self.wait_for_service(self.tf_set_mode_client_, '/tf_set_mode_command')
        found &= self.wait_for_service(self.tf_set_charge_mode_client_,
                                       '/tf_set_charge_mode_command')
        found &= self.wait_for_service(self.tf_set_curr_lim_client_, '/tf_set_curr_lim_command')
        found &= self.wait_for_service(self.tf_set_state_machine_client_,
                                       '/tf_set_state_machine_command')
        found &= self.wait_for_service(self.tf_watchdog_client_, '/tf_watchdog_command')
        found &= self.wait_for_service(self.tf_reset_client_, '/tf_reset_command')

        if not found:
            self.get_logger().warn('Did not find all services')
        return found

    def setup_subscribers(self):
        self.subs_ = []
        sub_info = []
        sub_info.append(['ahrs_callback', '/ahrs_data', XBRecord, self.ahrs_callback])
        sub_info.append(['battery_callback', '/battery_data', BCRecord, self.battery_callback])
        sub_info.append(['spring_callback', '/spring_data', SCRecord, self.spring_callback])
        sub_info.append(['power_callback', '/power_data', PCRecord, self.power_callback])
        sub_info.append(['trefoil_callback', '/trefoil_data', TFRecord, self.trefoil_callback])
        sub_info.append(['ahrs_callback', '/xb_record', XBRecord, self.ahrs_callback])
        sub_info.append(['battery_callback', '/bc_record', BCRecord, self.battery_callback])
        sub_info.append(['spring_callback', '/sc_record', SCRecord, self.spring_callback])
        sub_info.append(['power_callback', '/pc_record', PCRecord, self.power_callback])
        sub_info.append(['trefoil_callback', '/tf_record', TFRecord, self.trefoil_callback])
        sub_info.append(['powerbuoy_callback', '/powerbuoy_data',
                         PBRecord, self.powerbuoy_callback])
        sub_info.append(['latent_callback', '/latent_data',
                         LatentData, self.latent_callback])
        for cb_name, topic, msg_type, cb in sub_info:
            if cb_name in self.__class__.__dict__:  # did derived override a callback?
                self.get_logger().info("Subscribing to {msg_type} on '{topic}'".format(
                                       msg_type=str(msg_type), topic=topic))
                sub = self.create_subscription(msg_type, topic, cb, 10, callback_group=self.cb_sub)
                self.subs_.append(sub)

    def use_sim_time(self, enable=True):
        """
        Enable/Disable using sim time in Node clock from /clock.

        :param bool enable: True to use /clock, False to use system time
        """
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, enable)])

    def set_pc_pack_rate_param(self, rate_hz=50.0, blocking=True, timeout=None):
        """
        Set publish rate of PC Microcontroller telemetry.

        :param float rate_hz: desired publish rate in Hz
        :param bool blocking: if True, wait for the service call to complete
        """
        request = SetParameters.Request()
        request.parameters = [Parameter(name='publish_rate',
                                        value=float(rate_hz)).to_parameter_msg()]
        self.pc_pack_rate_param_future_ = self.pc_pack_rate_param_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pc_pack_rate_param_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('pc pack rate command failed')

    def set_sc_pack_rate_param(self, rate_hz=50.0, blocking=True, timeout=None):
        """
        Set publish rate of SC Microcontroller telemetry.

        :param float rate_hz: desired publish rate in Hz
        :param bool blocking: if True, wait for the service call to complete
        """
        request = SetParameters.Request()
        request.parameters = [Parameter(name='publish_rate',
                                        value=float(rate_hz)).to_parameter_msg()]
        self.sc_pack_rate_param_future_ = self.sc_pack_rate_param_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.sc_pack_rate_param_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('sc pack rate command failed')

    # set publish rate of PC Microcontroller telemetry
    def set_pc_pack_rate(self, rate_hz=50, blocking=True, timeout=None):
        """
        Set publish rate of PC Microcontroller telemetry.

        :param float rate_hz: desired publish rate in Hz
        :param bool blocking: if True, wait for the service call to complete
        """
        request = PCPackRateCommand.Request()
        request.rate_hz = int(rate_hz)

        self.pc_pack_rate_future_ = self.pc_pack_rate_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pc_pack_rate_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('pc pack rate command failed')

    # set publish rate of SC Microcontroller telemetry
    def set_sc_pack_rate(self, rate_hz=50, blocking=True, timeout=None):
        """
        Set publish rate of SC Microcontroller telemetry.

        :param float rate_hz: desired publish rate in Hz
        :param bool blocking: if True, wait for the service call to complete
        """
        request = SCPackRateCommand.Request()
        request.rate_hz = int(rate_hz)

        self.sc_pack_rate_future_ = self.sc_pack_rate_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.sc_pack_rate_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('sc pack rate command failed')

    def send_pump_command(self, duration_mins, blocking=True, timeout=None):
        """
        Turn pump on for a duration in minutes to raise mean piston position.

        :param float duration_mins: pump on duration in minutes
        :param bool blocking: if True, wait for the service call to complete
        """
        request = PumpCommand.Request()
        request.duration_mins = float(duration_mins)

        self.pump_future_ = self.pump_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pump_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('pump command failed')

    def send_valve_command(self, duration_sec, blocking=True, timeout=None):
        """
        Turn valve on for a duration in seconds to lower mean piston position.

        :param float duration_sec: valve on duration in seconds
        :param bool blocking: if True, wait for the service call to complete
        """
        request = ValveCommand.Request()
        request.duration_sec = int(duration_sec)

        self.valve_future_ = self.valve_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.valve_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('valve command failed')

    def send_pc_wind_curr_command(self, wind_curr, blocking=True, timeout=None):
        """
        Set winding current setpoint to control piston damping.

        :param float wind_curr: wind current setpoint in Amps
        :param bool blocking: if True, wait for the service call to complete
        """
        request = PCWindCurrCommand.Request()
        request.wind_curr = float(wind_curr)

        self.pc_wind_curr_future_ = self.pc_wind_curr_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pc_wind_curr_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('pc winding current command failed')

    def send_pc_bias_curr_command(self, bias_curr, blocking=True, timeout=None):
        """
        Set bias current setpoint to control piston damping offset.

        A High bias in either direction will move the piston back and forth

        :param float bias_curr: bias current setpoint in Amps
        :param bool blocking: if True, wait for the service call to complete
        """
        request = PCBiasCurrCommand.Request()
        request.bias_curr = float(bias_curr)

        self.pc_bias_curr_future_ = self.pc_bias_curr_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pc_bias_curr_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('pc bias current command failed')

    def send_pc_scale_command(self, scale, blocking=True, timeout=None):
        """
        Set damping gain.

        :param float scale: damping gain
        :param bool blocking: if True, wait for the service call to complete
        """
        request = PCScaleCommand.Request()
        request.scale = float(scale)

        self.pc_scale_future_ = self.pc_scale_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pc_scale_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('pc scale command failed')

    def send_pc_retract_command(self, retract, blocking=True, timeout=None):
        """
        Set additional damping gain in the piston retract direction.

        :param float retract: additional damping gain for retraction
        :param bool blocking: if True, wait for the service call to complete
        """
        done = threading.Event()
        request = PCRetractCommand.Request()
        request.retract = float(retract)

        self.pc_retract_future_ = self.pc_retract_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.pc_retract_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if blocking:
            if not done.wait(timeout) or 'except' in resp_dict:
                self.get_logger().error('retract command failed')

    def get_inc_wave_height(self, x, y, t, use_buoy_origin, use_relative_time, timeout=2.0):
        """
        Request incident wave height at specific location(s) and time(s).

        :param float or list x: x position(s) of desired wave height(s)
        :param float or list y: y position(s) of desired wave height(s)
        :param float or list t: t time(s) of desired wave height(s)
        :param bool use_buoy_origin: if True, x and y are relative to buoy origin
        :param bool use_relative_time: if True, desired time(s) are relative to current sim time
                                       else, time(s) will be absolute sim time
        :param float timeout: if not None, wait for timeout sec for the service call to complete
                              else, wait forever
        """
        request = IncWaveHeight.Request()
        request.use_buoy_origin = use_buoy_origin
        request.use_relative_time = use_relative_time

        xn = np.atleast_1d(x)
        yn = np.atleast_1d(y)
        tn = np.atleast_1d(t)

        for x, y, t in zip(xn, yn, tn):
            p = Point()
            p.x = x
            p.y = y
            request.points.append(p)
            if use_relative_time:
                request.relative_time.append(t)
            else:
                request.absolute_time.append(t)

        self.iwh_future_ = self.iwh_client_.call_async(request)
        done = threading.Event()
        resp_dict = {}
        self.iwh_future_.add_done_callback(
            partial(
                self.default_service_response_callback,
                done=done,
                timeout=timeout,
                resp_dict=resp_dict
            )
        )

        if not done.wait(timeout) or 'except' in resp_dict:
            self.get_logger().error('inc wave height request failed')
            return None

        if 'resp' in resp_dict:
            return resp_dict['resp']

        return None

    # set_params and callbacks optionally defined by user
    def set_params(self):
        """Set user-defined Node params (e.g. custom controller gains)."""
        pass

    def ahrs_callback(self, data):
        """
        Override this function to subscribe to /ahrs_data to receive XBRecord telemetry.

        :param data: incoming XBRecord
        """
        pass

    def battery_callback(self, data):
        """
        Override this function to subscribe to /battery_data to receive BCRecord telemetry.

        :param data: incoming BCRecord
        """
        pass

    def spring_callback(self, data):
        """
        Override this function to subscribe to /spring_data to receive SCRecord telemetry.

        :param data: incoming SCRecord
        """
        pass

    def power_callback(self, data):
        """
        Override this function to subscribe to /power_data to receive PCRecord telemetry.

        :param data: incoming PCRecord
        """
        pass

    def trefoil_callback(self, data):
        """
        Override this function to subscribe to /trefoil_data to receive TFRecord telemetry.

        :param data: incoming TFRecord
        """
        pass

    def powerbuoy_callback(self, data):
        """
        Override this function to subscribe to /powerbuoy_data to receive PBRecord telemetry.

        PBRecord contains a slice of all microcontroller's telemetry data

        :param data: incoming PBRecord
        """
        pass

    def latent_callback(self, data):
        """
        Override this function to subscribe to /latent_data to receive sim-only LatentData.

        :param data: incoming LatentData
        """
        pass

    def param_response_callback(self, future):
        resp = future.result()
        self.get_logger().info(f'Set Param Result: {resp}')
        # if resp.successful:
        #     self.get_logger().info('Successfully set param.')
        # else:
        #     self.get_logger().error('Param not set.')

    # generic service callback
    def default_service_response_callback(self, future, done, timeout, resp_dict):
        resp = None
        try:
            resp = future.result()
            if resp.result.value == resp.result.OK:
                self.get_logger().debug('Command Successful')
            else:
                self.get_logger().error(
                  f'Command Failed: received error code [[ {pbsrv_enum2str[resp.result.value]} ]]')
                # TODO(andermi): should we shutdown?

            resp_dict['resp'] = resp
        except concurrent.futures.TimeoutError as err:
            resp_dict['except'] = err
        finally:
            done.set()

    def wait_for_service(self, client, service_name, _count=1):
        count = 0
        while count < _count and not client.wait_for_service(timeout_sec=0.1):
            count += 1
            if not rclpy.ok():
                self.get_logger().error(
                  'Interrupted while waiting for {sn}. Exiting.'.format(sn=service_name))
                return False

            self.get_logger().info(
              '{sn} not available'.format(sn=service_name))
        return count < _count
