#!/usr/bin/python3

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

from buoy_api import Interface
import rclpy
from scipy import interpolate


class NLBiasDampingPolicy(object):

    def __init__(self):
        self.breaks = [0.0, 2.03]  # TODO(andermi) find suitable defaults (or leave this)
        self.bias = [0.0, 0.0]  # TODO(andermi) find suitable defaults (or leave this)
        self.deadzone = [-10.0, 90.0]  # TODO(andermi) find suitable defaults
        self.bias_interp1d = None
        self.update_params()

    def update_params(self):
        if len(self.breaks) >= 2 and len(self.bias) >= 2:
            self.bias_interp1d = interpolate.interp1d(self.breaks, self.bias,
                                                      fill_value=(0.0, self.bias[-1]),
                                                      bounds_error=False)
        else:
            self.bias_interp1d = None

    def bias_current_target(self, piston_pos):
        if self.deadzone[0] < piston_pos < self.deadzone[1]:
            return None
        if self.bias_interp1d is not None:
            return self.bias_interp1d(piston_pos)
        else:
            return None

    def __str__(self):
        return """NLBiasDampingPolicy:
\tposition_breaks: {breaks}
\tbias: {bias}
\tposition_deadzone: {deadzone}""".format(breaks=self.breaks,
                                          bias=self.bias,
                                          deadzone=self.deadzone)


class NonLinearBiasDamping(Interface):

    def __init__(self):
        super().__init__('pb_nl_bias_damping')
        self.policy = NLBiasDampingPolicy()
        self.set_params()
        print(self.policy)
        self.set_sc_pack_rate_param()

    def set_params(self):
        self.declare_parameter('bias_damping.position_breaks', self.policy.breaks)
        self.declare_parameter('bias_damping.bias', self.policy.bias)
        self.declare_parameter('bias_damping.position_deadzone', self.policy.deadzone)
        position_break_params = self.get_parameters_by_prefix('bias_damping')

        self.policy.breaks = \
            position_break_params['position_breaks'].get_parameter_value().double_array_value
        self.policy.bias = \
            position_break_params['bias'].get_parameter_value().double_array_value
        self.policy.deadzone = \
            position_break_params['position_deadzone'].get_parameter_value().double_array_value

        self.policy.update_params()

    def spring_callback(self, data):
        bct = self.policy.bias_current_target(data.range_finder)
        self.get_logger().info(f'Bias Damping: f({data.range_finder}) = {bct}')
        if bct is None:
            return

        self.send_pc_bias_curr_command(float(bct), blocking=False)


def main():
    # import threading
    rclpy.init()
    controller = NonLinearBiasDamping()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(controller)
    # exe_t = threading.Thread(target=executor.spin, daemon=True)
    # exe_t.start()
    # rate = controller.create_rate(2)
    # while rclpy.ok():
    #     rate.sleep()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
