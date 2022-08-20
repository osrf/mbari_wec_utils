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
import time

from buoy_api import Interface

import rclpy


class PBCmd(Interface):

    def __init__(self):
        rclpy.init()
        super().__init__('pbcmd')
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.spin_thread.daemon = True
        self.spin_thread.start()


def float_range(mini, maxi):
    def float_range_checker(arg):
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
    return float_range_checker


def int_or_off(arg):
    try:
        return int(arg)
    except ValueError:
        if arg.lower() == 'off':
            return arg.lower()
        raise argparse.ArgumentTypeError("argument must be an int or 'off'")


def pbcmd(parser):
    print("""pbcmd: Multi-call command Power Buoy dispatcher
Supported commands:
*             bender  - Sets the state of the bender module
*      reset_battery  - Reset battery controller (caution - no args)

*               pump  - Spring Controller pump  off or on for a time in minutes <= 127
*              valve  - Spring Controller valve off or on for a time in seconds <= 127
*             tether  - Spring Controller tether power on or off
*       sc_pack_rate  - Set the CANBUS packet rate from the spring controller
*       reset_spring  - Reset Spring Controller (caution - no args)

*           pc_Scale  - Set the scale factor
*         pc_Retract  - Set the retract factor
*        pc_VTargMax  - Set the max target voltage
*   pc_ChargeCurrLim  - Set the maximum battery charge current
*     pc_DrawCurrLim  - Set the maximum battery current draw
*      pc_BattSwitch  - Set the battery switch state
*            pc_Gain  - Set the gain scheduler gain
*      pc_StdDevTarg  - Set the target RPM standard deviation
*        pc_WindCurr  - Set the winding current target
*        pc_BiasCurr  - Set the winding current bias
*        pc_PackRate  - Set the CANBUS packet rate

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
    parser.add_argument('duration_minutes', type=int_or_off,
                        choices=tuple(['off'] + list(range(127))),
                        help='Executing pump to Spring Controller pump ' +
                             'off or on for a time in minutes <= 127...',
                        metavar='duration_minutes')
    args = parser.parse_args()
    duration_minutes = args.duration_minutes
    if duration_minutes == 'off':
        print(f'Executing pump to Spring Controller pump: {duration_minutes}')
        duration_minutes = 0
    else:
        print(f'Executing pump to Spring Controller pump: {duration_minutes} minute(s)')
    _pbcmd = PBCmd()
    _pbcmd.send_pump_command(duration_minutes * 60)
    time.sleep(1)
    print('done!')


def valve(parser):
    parser.add_argument('duration_sec', type=int_or_off,
                        choices=tuple(['off'] + list(range(12))),
                        help='Executing valve to Spring Controller valve ' +
                             'off or on for a time in seconds <= 12...',
                        metavar='duration_sec')
    args = parser.parse_args()
    duration_sec = args.duration_sec
    if duration_sec == 'off':
        print(f'Executing valve to Spring Controller valve: {duration_sec}')
        duration_sec = 0
    else:
        print(f'Executing valve to Spring Controller valve: {duration_sec} second(s)')
    _pbcmd = PBCmd()
    _pbcmd.send_valve_command(duration_sec)
    time.sleep(1)
    print('done!')


def main():
    parser = argparse.ArgumentParser()

    cmds = {'pbcmd': pbcmd,
            'pump': pump,
            'valve': valve}

    if parser.prog in cmds:
        cmds[parser.prog](parser)
    else:
        print(parser.prog + ' not currently implemented')


if __name__ == '__main__':
    main()
