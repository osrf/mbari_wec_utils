#!/usr/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ALIAS_INSTALL_DIR=/usr/local/bin

ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pbcmd
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/bender
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/reset_battery
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pump
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/valve
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tether
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/reset_spring
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/sc_pack_rate

ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_Scale
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_Retract
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_VTargMax
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_ChargeCurrLim
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_Gain
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_StdDevTarg
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_DrawCurrLim
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_BattSwitch
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_WindCurr
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_BiasCurr
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/pc_PackRate

ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_SetPos
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_SetActualPos
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_SetMode
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_SetChargeMode
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_SetStateMachine
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_SetCurrLim
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_WatchDog
ln -sf $SCRIPT_DIR/pbcmd $ALIAS_INSTALL_DIR/tf_Reset
