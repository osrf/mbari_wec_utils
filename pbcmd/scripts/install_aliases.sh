#!/usr/bin/bash
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

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ALIAS_INSTALL_DIR=$HOME/.local/bin
mkdir -p $ALIAS_INSTALL_DIR

# Ensure ~/.profile exists to pick up
# ~/.local/bin for all the pbcmd aliases
SKEL_PROFILE=/etc/skel/.profile
HOME_PROFILE=$HOME/.profile
if [ ! -f "$HOME_PROFILE" ]; then
    cp -v $SKEL_PROFILE $HOME_PROFILE
fi

cmds=(pbcmd bender reset_battery pump valve tether reset_spring sc_pack_rate)
cmds+=(pc_Scale pc_Retract pc_VTargMax pc_ChargeCurrLim pc_Gain pc_StdDevTarg)
cmds+=(pc_DrawCurrLim pc_BattSwitch pc_WindCurr pc_BiasCurr pc_PackRate)
cmds+=(tf_SetPos tf_SetActualPos tf_SetMode tf_SetChargeMode tf_SetStateMachine)
cmds+=(tf_SetCurrLim tf_WatchDog tf_Reset)

echo "installing pbcmd aliases to" $ALIAS_INSTALL_DIR

for index in "${!cmds[@]}"; do
    echo "ln -sf" $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/"${cmds[$index]}"
    ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/"${cmds[$index]}"
done


