# mbari_wec_utils

Packages in this repo provide interfaces, API, examples for [MBARI WEC](https://github.com/osrf/mbari_wec/tree/v1.1.0).

Complete examples starting from template repositories may be found here:
[Python linear damper](https://github.com/mbari-org/mbari_wec_template_py/tree/linear_damper_example)
[C++ linear damper](https://github.com/mbari-org/mbari_wec_template_cpp/tree/linear_damper_example)

And tutorials may be found [here](https://osrf.github.io/mbari_wec/v1.1.0/tutorials).

For more information about ROS 2 interfaces, see [docs.ros.org](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html).

## Messages (.msg)
Collection of telemetry (feedback) from buoy sensors and microcontrollers.

### Telemetry
Telemetry data from sensors available per microcontroller

- type: `BCRecord`
  topic: `/bc_record`
  Description: (TODO) battery data

- type: `PCRecord`
  topic: `/pc_record`
  Description: (TODO) power controller data

- type: `SCRecord`
  topic: `/sc_record`
  Description: (TODO) spring controller data

- type: `TFRecord`
  topic: `/tf_record`
  Description: (TODO) trefoil data

- type: `XBRecord`
  topic: `/xb_record`
  Description: (TODO) Crossbow AHRS data

### Consolidated (Time-Synchronized) Telemetry
Snapshot in time of all data from the buoy.

- type: `PBRecord`
  topic: `/pb_record`
  Description: TODO

## Services (.srv)
Collection of commands to configure/control the Power Buoy.

### Power Microcontroller
Services available to configure the Power Microcontroller.

- type: `PCBattSwitchCommand`
  topic: `/pc_batt_switch_command`
  args: OFF, ON (enum)
  Description: TODO

- type: `PCBiasCurrCommand`
  topic: `/pc_bias_curr_command`
  args: -15.0 to +15.0 (float, amps)
  Description: TODO

- type: `PCChargeCurrLimCommand`
  topic: `/pc_charge_curr_lim_command`
  args: 2.0 to 12.0 (float, amps)
  Description: TODO

- type: `PCDrawCurrLimCommand`
  topic: `/pc_draw_curr_lim_command`
  args: 2.0 to 12.0 (float, amps)
  Description: TODO

- type: `PCPackRateCommand`
  topic: `/pc_pack_rate_command`
  args: 10 to 50 (int, hz)
  Description: set PCRecord publish rate in Hz

- type: `PCRetractCommand`
  topic: `/pc_retract_command`
  args: 0.4 to 1.0 (float)
  Description: TODO

- type: `PCScaleCommand`
  topic: `/pc_scale_command`
  args: 0.5 to 1.4 (float)
  Description: TODO

- type: `PCStdDevTargCommand`
  topic: `/pc_std_dev_targ_command`
  args: 0 to 2000 (int)
  Description: TODO

- type: `PCVTargMaxCommand`
  topic: `/pc_v_targ_max_command`
  args: 320.0 to 330.0 (float, volts)
  Description: TODO

- type: `PCWindCurrCommand`
  topic: `/pc_wind_curr_command`
  args: -35.0 to 35.0 (float, amps)
  Description: TODO

- type: `GainCommand`
  topic: `/gain_command`
  args: OFF, 1 to 200 (enum or int)
  Description: TODO

### Battery Microcontroller
Services available to configure the Battery Microcontroller.

- type: `BCResetCommand`
  topic: `/bc_reset_command`
  args: None
  Description: TODO

### Spring Microcontroller
- type: `SCPackRateCommand`
  topic: `/sc_pack_rate_command`
  args: 10 to 50 (int, hz)
  Description: set SCRecord publish rate in Hz

- type: `SCResetCommand`
  topic: `/sc_reset_command`
  args: None
  Description: TODO

- type: `ValveCommand`
  topic: `/valve_command`
  args: OFF, 1 to 127 (enum or int, seconds)
  Description: TODO

- type: `PumpCommand`
  topic: `/pump_command`
  args: OFF, 1 to 127 (enum or int, seconds)
  Description: TODO

- type: `BenderCommand`
  topic: `/bender_command`
  args: OFF, ON, AUTO (enum)
  Description: TODO

- type: `TetherCommand`
  topic: `/tether_command`
  args: OFF, ON (enum)
  Description: TODO

### Trefoil Microcontroller
Services available to configure the Trefoil.

- type: `TFResetCommand`
  topic: `/tf_reset_command`
  args: None
  Description: TODO

- type: `TFSetActualPosCommand`
  topic: `/tf_set_actual_pos_command`
  args: OPEN, CLOSED (enum)
  Description: TODO

- type: `TFSetChargeModeCommand`
  topic: `/tf_set_charge_mode_command`
  args: OFF, ON, AUTO_TIME, AUTO_VOLTAGE (enum)
  Description: TODO

- type: `TFSetCurrLimCommand`
  topic: `/tf_set_curr_lim_command`
  args: 500 to 5000 (int, TODO(units))
  Description: TODO

- type: `TFSetModeCommand`
  topic: `/tf_set_mode_command`
  args: POWER_10W, MONITOR_POS, HOLD_POS (enum)
  Description: TODO

- type: `TFSetPosCommand`
  topic: `/tf_set_pos_command`
  args: OPEN, CLOSED (enum)
  Description: TODO

- type: `TFSetStateMachineCommand`
  topic: `/tf_set_state_machine_command`
  args: HOME_MOVE, VEL_MOVE (enum)
  Description: TODO

- type: `TFWatchDogCommand`
  topic: `/tf_watchdog_command`
  args: None
  Description: TODO


## Quality Declaration

This package claims to be in the **Quality Level 5** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
