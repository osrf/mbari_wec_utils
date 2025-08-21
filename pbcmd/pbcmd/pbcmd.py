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

import argparse
import threading

from buoy_api import Interface

import rclpy


class _PBCmd(Interface):

    def __init__(self):
        rclpy.init()
        super().__init__('pbcmd', check_for_services=False)
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()


def _float_range(mini, maxi):
    def _float_range_checker(arg):
        try:
            f = float(arg)
        except ValueError:
            raise argparse.ArgumentTypeError('must be a floating point number')
        if f < mini or f > maxi:
            raise argparse.ArgumentTypeError('must be in range [' +
                                             str(mini) + ' .. ' +
                                             str(maxi)+']')
        return f

    # Return function handle to checking function
    return _float_range_checker


def _float_or_off(mini, maxi):
    def _float_or_off_checker(arg):
        f = _float_range(mini, maxi)
        try:
            return f(arg)
        except argparse.ArgumentTypeError as err:
            if 'range' in err.args[0]:
                raise err
            if arg.lower() == 'off':
                return arg.lower()
            raise argparse.ArgumentTypeError("argument must be a float or 'off'")

    return _float_or_off_checker


def _int_or_off(arg):
    try:
        return int(arg)
    except ValueError:
        if arg.lower() == 'off':
            return arg.lower()
        raise argparse.ArgumentTypeError("argument must be an int or 'off'")


def pbcmd(parser):
    print("""pbcmd: Multi-call command Power Buoy dispatcher
Supported commands (Simulation):
*               pump  - Spring Controller pump  off or on for a time in minutes <= 10
*              valve  - Spring Controller valve off or on for a time in seconds <= 12
*       sc_pack_rate  - Set the CANBUS packet rate from the spring controller
*           pc_Scale  - Set the scale factor
*         pc_Retract  - Set the retract factor
*        pc_WindCurr  - Set the winding current target
*        pc_BiasCurr  - Set the winding current bias
*        pc_PackRate  - Set the CANBUS packet rate

Curently unsupported commands (Simulation):
*             bender  - Sets the state of the bender module
*      reset_battery  - Reset battery controller (caution - no args)

*             tether  - Spring Controller tether power on or off
*       reset_spring  - Reset Spring Controller (caution - no args)

*        pc_VTargMax  - Set the max target voltage
*   pc_ChargeCurrLim  - Set the maximum battery charge current
*     pc_DrawCurrLim  - Set the maximum battery current draw
*      pc_BattSwitch  - Set the battery switch state
*            pc_Gain  - Set the gain scheduler gain
*      pc_StdDevTarg  - Set the target RPM standard deviation

*          tf_SetPos  - Open/close the doors in the heave-cone
*    tf_SetActualPos  - Open/close the doors in the heave-cone
*         tf_SetMode  - Set controller mode
*   tf_SetChargeMode  - Set Battery Charge mode
* tf_SetStateMachine  - Set Battery Charge mode
*      tf_SetCurrLim  - Set controller current limit
*        tf_WatchDog  - Toggle controller watchdog (caution - no args)
*           tf_Reset  - Reset Controller (caution - no args)


For help on a command use the command name, i.e., "bender"

Except the reset commands which take no arguments.

DO NOT enter reset_power and expect to get help. The command will execute!""")


def pump(parser):
    parser.add_argument('duration_minutes', type=_float_or_off(0, 10),
                        help='(Minutes) [off, <= 10] ' +
                             'Spring Controller pump off or on for a time in minutes',
                        metavar='duration_minutes')
    args = parser.parse_args()
    duration_minutes = args.duration_minutes
    if duration_minutes == 'off':
        print(f'Executing pump to Spring Controller: {duration_minutes}')
        duration_minutes = 0
    else:
        print(f'Executing pump to Spring Controller: {duration_minutes} minute(s)')
    _pbcmd = _PBCmd()
    _pbcmd.send_pump_command(duration_minutes)


def valve(parser):
    parser.add_argument('duration_sec', type=_int_or_off,
                        choices=tuple(['off'] + list(range(13))),
                        help='(Seconds) [off, <= 12] ' +
                             'Spring Controller valve off or on for a time in seconds',
                        metavar='duration_sec')
    args = parser.parse_args()
    duration_sec = args.duration_sec
    if duration_sec == 'off':
        print(f'Executing valve to Spring Controller: {duration_sec}')
        duration_sec = 0
    else:
        print(f'Executing valve to Spring Controller: {duration_sec} second(s)')
    _pbcmd = _PBCmd()
    _pbcmd.send_valve_command(duration_sec)


def sc_pack_rate(parser):
    parser.add_argument('rate_hz', type=int,
                        choices=tuple(range(10, 51)),
                        help='(Hz) [10 to 50] ' +
                             'Set the CANBUS packet rate from the Spring Controller',
                        metavar='rate_hz')
    args = parser.parse_args()
    print('Executing sc_pack_rate to Set the CANBUS packet rate ' +
          f'from the Spring Controller: {args.rate_hz} Hz')
    _pbcmd = _PBCmd()
    _pbcmd.set_sc_pack_rate(args.rate_hz)


def pc_PackRate(parser):
    parser.add_argument('rate_hz', type=int,
                        choices=tuple(range(10, 51)),
                        help='(Hz) [10 to 50] ' +
                             'Set the CANBUS packet rate from the Power Controller',
                        metavar='rate_hz')
    args = parser.parse_args()
    print('Executing pc_PackRate to Set the CANBUS packet rate ' +
          f'from the Power Controller: {args.rate_hz} Hz')
    _pbcmd = _PBCmd()
    _pbcmd.set_pc_pack_rate(args.rate_hz)


def pc_Scale(parser):
    parser.add_argument('scale', type=_float_range(0.5, 1.4),
                        help='[0.5 to 1.4] ' +
                             'Set the scale factor for Power Controller winding current',
                        metavar='scale')
    args = parser.parse_args()
    print('Executing pc_Scale to Set the scale factor for ' +
          f'Power Controller winding current: {args.scale}')
    _pbcmd = _PBCmd()
    _pbcmd.send_pc_scale_command(args.scale)


def pc_Retract(parser):
    parser.add_argument('retract', type=_float_range(0.4, 1.0),
                        help='[0.5 to 1.0] ' +
                             'Set the retract factor for Power Controller winding current',
                        metavar='retract')
    args = parser.parse_args()
    print('Executing pc_Retract to Set the retract factor for ' +
          f'Power Controller winding current: {args.retract}')
    _pbcmd = _PBCmd()
    _pbcmd.send_pc_retract_command(args.retract)


def pc_WindCurr(parser):
    parser.add_argument('wind_curr', type=_float_range(-35.0, 35.0),
                        help='(Amps) [-35.0 to 35.0] ' +
                             'Set the winding current target for Power Controller',
                        metavar='wind_curr')
    args = parser.parse_args()
    print('Executing pc_WindCurr to Set the winding current target for ' +
          f'Power Controller: {args.wind_curr}')
    _pbcmd = _PBCmd()
    _pbcmd.send_pc_wind_curr_command(args.wind_curr)


def pc_BiasCurr(parser):
    parser.add_argument('bias_curr', type=_float_range(-15.0, 15.0),
                        help='(Amps) [-15.0 to 15.0] ' +
                             'Set the bias for Power Controller winding current',
                        metavar='bias_curr')
    args = parser.parse_args()
    print('Executing pc_BiasCurr to Set the bias for ' +
          f'Power Controller winding current: {args.bias_curr}')
    _pbcmd = _PBCmd()
    _pbcmd.send_pc_bias_curr_command(args.bias_curr)


def main():
    parser = argparse.ArgumentParser()

    cmds = {'pbcmd': pbcmd,
            'pump': pump,
            'valve': valve,
            'sc_pack_rate': sc_pack_rate,
            'pc_Scale': pc_Scale,
            'pc_Retract': pc_Retract,
            'pc_WindCurr': pc_WindCurr,
            'pc_BiasCurr': pc_BiasCurr,
            'pc_PackRate': pc_PackRate}

    if parser.prog in cmds:
        cmds[parser.prog](parser)
    else:
        print(parser.prog + ' not currently implemented')


if __name__ == '__main__':
    main()
