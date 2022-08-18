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

import asyncio

# pbsrv commands
# power microcontroller
from buoy_msgs.srv import GainCommand  # noqa
from buoy_msgs.srv import PCBattSwitchCommand  # noqa
from buoy_msgs.srv import PCBiasCurrCommand  # noqa
from buoy_msgs.srv import PCChargeCurrLimCommand  # noqa
from buoy_msgs.srv import PCDrawCurrLimCommand  # noqa
from buoy_msgs.srv import PCPackRateCommand  # noqa
from buoy_msgs.srv import PCRetractCommand  # noqa
from buoy_msgs.srv import PCScaleCommand  # noqa
from buoy_msgs.srv import PCStdDevTargCommand  # noqa
from buoy_msgs.srv import PCVTargMaxCommand  # noqa
from buoy_msgs.srv import PCWindCurrCommand  # noqa

# battery microcontroller
from buoy_msgs.srv import BCResetCommand  # noqa

# spring microcontroller
from buoy_msgs.srv import BenderCommand  # noqa
from buoy_msgs.srv import PumpCommand  # noqa
from buoy_msgs.srv import SCResetCommand  # noqa
from buoy_msgs.srv import SCPackRateCommand  # noqa
from buoy_msgs.srv import TetherCommand  # noqa
from buoy_msgs.srv import ValveCommand  # noqa

# trefoil microcontroller
from buoy_msgs.srv import TFResetCommand  # noqa
from buoy_msgs.srv import TFSetActualPosCommand  # noqa
from buoy_msgs.srv import TFSetChargeModeCommand  # noqa
from buoy_msgs.srv import TFSetCurrLimCommand  # noqa
from buoy_msgs.srv import TFSetModeCommand  # noqa
from buoy_msgs.srv import TFSetPosCommand  # noqa
from buoy_msgs.srv import TFSetStateMachineCommand  # noqa
from buoy_msgs.srv import TFWatchDogCommand  # noqa


# pb telemetry
from buoy_msgs.msg import BCRecord  # battery  # noqa
from buoy_msgs.msg import PBRecord  # consolidated  # noqa
from buoy_msgs.msg import PCRecord  # power  # noqa
from buoy_msgs.msg import SCRecord  # spring  # noqa
from buoy_msgs.msg import TFRecord  # trefoil  # noqa
from buoy_msgs.msg import XBRecord  # ahrs  # noqa


# Pack Rate Params
from rclpy.parameter import Parameter  # noqa
from rcl_interfaces.srv import SetParameters  # noqa


import rclpy
from rclpy.node import Node


pbsrv_enum2str = {0: 'OK',
                  -1: 'BAD_SOCK',
                  -2: 'BAD_OPTS',
                  -3: 'BAD_INPUT'}


class Interface(Node):

    def __init__(self, node_name, wait_for_services=False, **kwargs):
        super().__init__(node_name, **kwargs)
        self.pc_pack_rate_param_client_ = self.create_client(SetParameters,
                                                             '/power_controller/set_parameters')
        self.pc_pack_rate_client_ = self.create_client(PCPackRateCommand, '/pc_pack_rate_command')
        self.pc_wind_curr_client_ = self.create_client(PCWindCurrCommand, '/pc_wind_curr_command')
        self.bender_client_ = self.create_client(BenderCommand, '/bender_command')
        self.bc_reset_client_ = self.create_client(BCResetCommand, '/bc_reset_command')
        self.pump_client_ = self.create_client(PumpCommand, '/pump_command')
        self.valve_client_ = self.create_client(ValveCommand, '/valve_command')
        self.tether_client_ = self.create_client(TetherCommand, '/tether_command')
        self.sc_reset_client_ = self.create_client(SCResetCommand, '/sc_reset_command')
        self.sc_pack_rate_param_client_ = self.create_client(SetParameters,
                                                             '/spring_controller/set_parameters')
        self.sc_pack_rate_client_ = self.create_client(SCPackRateCommand, '/sc_pack_rate_command')
        self.pc_scale_client_ = self.create_client(PCScaleCommand, '/pc_scale_command')
        self.pc_retract_client_ = self.create_client(PCRetractCommand, '/pc_retract_command')
        self.pc_v_targ_max_client_ = self.create_client(PCVTargMaxCommand,
                                                        '/pc_v_targ_max_command')
        self.pc_charge_curr_lim_client_ = self.create_client(PCChargeCurrLimCommand,
                                                             '/pc_charge_curr_lim_command')
        self.pc_batt_switch_client_ = self.create_client(PCBattSwitchCommand,
                                                         '/pc_batt_switch_command')
        self.gain_client_ = self.create_client(GainCommand, '/gain_command')
        self.pc_std_dev_targ_client_ = self.create_client(PCStdDevTargCommand,
                                                          '/pc_std_dev_targ_command')
        self.pc_draw_curr_lim_client_ = self.create_client(PCDrawCurrLimCommand,
                                                           '/pc_draw_curr_lim_command')
        self.pc_bias_curr_client_ = self.create_client(PCBiasCurrCommand, '/pc_bias_curr_command')
        self.tf_set_pos_client_ = self.create_client(TFSetPosCommand, '/tf_set_pos_command')
        self.tf_set_actual_pos_client_ = self.create_client(TFSetActualPosCommand,
                                                            '/tf_set_actual_pos_command')
        self.tf_set_mode_client_ = self.create_client(TFSetModeCommand, '/tf_set_mode_command')
        self.tf_set_charge_mode_client_ = self.create_client(TFSetChargeModeCommand,
                                                             '/tf_set_charge_mode_command')
        self.tf_set_curr_lim_client_ = self.create_client(TFSetCurrLimCommand,
                                                          '/tf_set_curr_lim_command')
        self.tf_set_state_machine_client_ = self.create_client(TFSetStateMachineCommand,
                                                               '/tf_set_state_machine_command')
        self.tf_watch_dog_client_ = self.create_client(TFWatchDogCommand, '/tf_watch_dog_command')
        self.tf_reset_client_ = self.create_client(TFResetCommand, '/tf_reset_command')

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
        self.tf_watch_dog_future_ = None
        self.tf_reset_future_ = None

        self.setup_subscribers()
        found = self.wait_for_services()
        if not found and wait_for_services:
            while rclpy.ok() and not self.wait_for_services():
                pass

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
        found &= self.wait_for_service(self.tf_watch_dog_client_, '/tf_watch_dog_command')
        found &= self.wait_for_service(self.tf_reset_client_, '/tf_reset_command')

        if not found:
            self.get_logger().error('Did not find required services')
        return found

    # if user has shadowed a callback in their user-derived class, this will use their
    # implementation if they did not define one, the subscriber will not be set up
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
        for cb_name, topic, msg_type, cb in sub_info:
            if cb_name in self.__class__.__dict__:  # did derived override a callback?
                self.get_logger().info("Subscribing to {msg_type} on '{topic}'".format(
                                       msg_type=str(msg_type), topic=topic))
                sub = self.create_subscription(msg_type, topic, cb, 10)
                self.subs_.append(sub)

    # set publish rate of PC Microcontroller telemetry
    def set_pc_pack_rate_param(self, rate_hz=50.0):
        request = SetParameters.Request()
        request.parameters = [Parameter(name='publish_rate',
                                        value=float(rate_hz)).to_parameter_msg()]
        self.pc_pack_rate_param_future_ = self.pc_pack_rate_param_client_.call_async(request)
        self.pc_pack_rate_param_future_.add_done_callback(self.param_response_callback)

    # set publish rate of SC Microcontroller telemetry
    def set_sc_pack_rate_param(self, rate_hz=50.0):
        request = SetParameters.Request()
        request.parameters = [Parameter(name='publish_rate',
                                        value=float(rate_hz)).to_parameter_msg()]
        self.sc_pack_rate_param_future_ = self.sc_pack_rate_param_client_.call_async(request)
        self.sc_pack_rate_param_future_.add_done_callback(self.param_response_callback)

    # set publish rate of PC Microcontroller telemetry
    def set_pc_pack_rate(self, rate_hz=50.0):
        request = PCPackRateCommand.Request()
        request.rate_hz = float(rate_hz)

        self.pc_pack_rate_future_ = self.pc_pack_rate_client_.call_async(request)
        self.pc_pack_rate_future_.add_done_callback(self.default_service_response_callback)

    # set publish rate of SC Microcontroller telemetry
    def set_sc_pack_rate(self, rate_hz=50.0):
        request = SCPackRateCommand.Request()
        request.rate_hz = float(rate_hz)

        self.sc_pack_rate_future_ = self.sc_pack_rate_client_.call_async(request)
        self.sc_pack_rate_future_.add_done_callback(self.default_service_response_callback)

    def send_pump_command(self, duration_sec):
        return asyncio.run(self._send_pump_command(duration_sec))

    async def _send_pump_command(self, duration_sec):
        request = PumpCommand.Request()
        request.duration_sec = int(duration_sec)

        self.pump_future_ = self.pump_client_.call_async(request)
        self.pump_future_.add_done_callback(self.default_service_response_callback)
        await self.pump_future_

    def send_valve_command(self, duration_sec):
        return asyncio.run(self._send_valve_command(duration_sec))

    async def _send_valve_command(self, duration_sec):
        request = ValveCommand.Request()
        request.duration_sec = int(duration_sec)

        self.valve_future_ = self.valve_client_.call_async(request)
        self.valve_future_.add_done_callback(self.default_service_response_callback)
        await self.valve_future_

    def send_pc_wind_curr_command(self, wind_curr):
        return asyncio.run(self._send_pc_wind_curr_command(wind_curr))

    async def _send_pc_wind_curr_command(self, wind_curr):
        request = PCWindCurrCommand.Request()
        request.wind_curr = float(wind_curr)

        self.pc_wind_curr_future_ = self.pc_wind_curr_client_.call_async(request)
        self.pc_wind_curr_future_.add_done_callback(self.default_service_response_callback)
        await self.pc_wind_curr_future_

    def send_pc_bias_curr_command(self, bias_curr):
        return asyncio.run(self._send_pc_bias_curr_command(bias_curr))

    async def _send_pc_bias_curr_command(self, bias_curr):
        request = PCBiasCurrCommand.Request()
        request.bias_curr = float(bias_curr)

        self.pc_bias_curr_future_ = self.pc_bias_curr_client_.call_async(request)
        self.pc_bias_curr_future_.add_done_callback(self.default_service_response_callback)
        await self.pc_bias_curr_future_

    def send_pc_scale_command(self, scale):
        return asyncio.run(self._send_pc_scale_command(scale))

    async def _send_pc_scale_command(self, scale):
        request = PCScaleCommand.Request()
        request.scale = float(scale)

        self.pc_scale_future_ = self.pc_scale_client_.call_async(request)
        self.pc_scale_future_.add_done_callback(self.default_service_response_callback)
        await self.pc_scale_future_

    def send_pc_retract_command(self, retract):
        return asyncio.run(self._send_pc_retract_command(retract))

    async def _send_pc_retract_command(self, retract):
        request = PCRetractCommand.Request()
        request.retract = float(retract)

        self.pc_retract_future_ = self.pc_retract_client_.call_async(request)
        self.pc_retract_future_.add_done_callback(self.default_service_response_callback)
        await self.pc_retract_future_

    # set_params and callbacks optionally defined by user
    def set_params(self): pass
    def ahrs_callback(self, data): pass
    def battery_callback(self, data): pass
    def spring_callback(self, data): pass
    def power_callback(self, data): pass
    def trefoil_callback(self, data): pass
    def powerbuoy_callback(self, data): pass

    def param_response_callback(self, future):
        resp = future.result()
        self.get_logger().info(f'Set Param Result: {resp}')
        # if resp.successful:
        #     self.get_logger().info('Successfully set param.')
        # else:
        #     self.get_logger().error('Param not set.')

    # generic service callback
    def default_service_response_callback(self, future):
        resp = future.result()
        if resp.result.value == resp.result.OK:
            self.get_logger().info('Command Successful')
        else:
            self.get_logger().error(
              f'Command Failed: received error code [[ {pbsrv_enum2str[resp.result.value]} ]]')
            # TODO(andermi): should we shutdown?

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
