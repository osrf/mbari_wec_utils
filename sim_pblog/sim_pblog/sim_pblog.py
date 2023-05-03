#!/usr/bin/python3

# Copyright 2022 Open Source Robotics Foundation,Inc. and Monterey Bay Aquarium Research Institute
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

from datetime import datetime, timedelta
import gzip
import math
import os

from buoy_api import Interface

from buoy_interfaces.msg import BCRecord
from buoy_interfaces.msg import PCRecord
from buoy_interfaces.msg import SCRecord
from buoy_interfaces.msg import TFRecord
from buoy_interfaces.msg import XBRecord

import rclpy

from tf_transformations import euler_from_quaternion


# Controller IDs used as the first value in each record
BatteryConID = 0
SpringConID = 1
PowerConID = 2
TrefoilConID = 4
CrossbowID = 5

# WECLogger - duplicate legacy pblog behavior substituting ROS2 topics for CANBus
# Description:
#     Init phase: (1) Subscribe to all WEC controllers (e.g., power controller,
#                     spring controller, etc)
#                 (2) Create and open CSV output log file, with header line, ready
#                     for writing log records
#     Run phase : (1) Controller topic callback functions receive topic messages
#                 (2) Callback functions write new log records with message data
#                 (3) Fresh log files are created and old files zipped every hour
#
#     Notes     : (1) There is always a trailing comma in all header and
#                     record sections


class WECLogger(Interface):

    def __init__(self, loghome, logdir=None):
        super().__init__('sim_pblog', check_for_services=False)
        self.start_time = datetime.now()
        self.logger_time = self.start_time

        self.loghome = os.path.expanduser(loghome)
        # Create new log folder at the start of this instance of the logger
        self.logdir = self.logdir_setup(logdir)
        self.logfile = None
        self.logfilename = None
        self.logfiletime = datetime.fromtimestamp(0)

        self.logfileinterval_sec = 60 * 60  # in seconds
        # Get params from config if available to set self.logfileinterval_sec
        self.set_params()

        self.pc_header = ''
        self.bc_header = ''
        self.xb_header = ''
        self.sc_header = ''
        self.tf_header = ''

    # Create and open a new log file
    # The system time is used to create a unique CSV log file name
    # Example: "2023.03.31T23-59-59.csv" would be created just before midnight on March 31st

    def logfile_setup(self):
        # close existing log file and zip it shut
        if (self.logfile is not None):
            self.close_zip_logfile()

        # Open new file in logdir using the logger_time (2023.03.23T13.09.54.csv)
        csv = self.logger_time.strftime('%Y.%m.%dT%I.%M.%S') + '.csv'
        self.logfilename = os.path.join(self.logdir, csv)
        self.logfile = open(self.logfilename, mode='w', encoding='utf-8')
        self.write_header()

        # Point a link called 'latest' to the new directory
        # Renaming a temporary link works as 'ln -sf'
        templink = os.path.join(self.logdir, '__bogus__')
        os.symlink(csv, templink)
        latest = os.path.join(self.logdir, 'latest_csv')
        os.rename(templink, latest)

        self.get_logger().info(f'New log file: {self.logfilename}')

    # Write CSV header by writing each controller header section
    def write_header(self):
        # Preamble/Header section column names first
        self.logfile.write('Source ID, Timestamp (epoch seconds),')

        # Write each header section - order matters!
        self.write_pc_header()
        self.write_bc_header()
        self.write_xb_header()
        self.write_sc_header()
        self.write_tf_header()
        self.write_eol()

    # Write a complete data record by writing each controller data section
    def write_record(self, source_id, data):
        self.update_logger_time(data)

        # Create a fresh logfile when interval time has passed
        if (self.logger_time > (self.logfiletime + timedelta(seconds=self.logfileinterval_sec))):
            self.logfiletime = self.logger_time
            self.logfile_setup()

        # Use epoch seconds from logger_time to write the record preamble/header
        self.logfile.write(f'{source_id}, {self.logger_time.timestamp():.3f}, ')

        # Pass data and delegate writing to section writers - order matters!
        self.write_pc(data)
        self.write_bc(data)
        self.write_xb(data)
        self.write_sc(data)
        self.write_tf(data)
        self.write_eol()

    # Increment logger_time using timestamp in data header
    def update_logger_time(self, data):
        # Timestamps in the data messages contain the time since the
        # start of the simulation beginning at zero
        micros = (1.0 * data.header.stamp.nanosec) / 1000.   # convert to micros
        delta = timedelta(seconds=data.header.stamp.sec, microseconds=micros)
        # Calculated logger time
        newtime = self.start_time + delta
        # Occasionally data messages arrive out of time sequence
        # Ensure logfile timestamps are always increasing
        if (newtime > self.logger_time):
            self.logger_time = newtime

    def write_eol(self):
        self.logfile.write('\n')

    # Close the current log file and zip it
    def close_zip_logfile(self):
        if self.logfile is not None and not self.logfile.closed:
            self.logfile.close()
            with open(self.logfilename, 'rb') as logf:
                with gzip.open(f'{self.logfilename}.gz', 'wb') as gzfile:
                    gzfile.writelines(logf)
                    self.get_logger().info(f'{self.logfilename} -> {self.logfilename}.gz')
            os.remove(self.logfilename)

            # Point a link called 'latest' to the new directory
            # Renaming a temporary link works as 'ln -sf'
            csv_gz = os.path.basename(self.logfilename) + '.gz'
            templink = os.path.join(self.logdir, '__bogus__')
            os.symlink(csv_gz, templink)
            latest = os.path.join(self.logdir, 'latest_csv')
            os.rename(templink, latest)

    # Create a new directory for log files created for this instance of logger
    # The system date is used to create a unique directory name for this run
    # Example: "2023-03-31.005 would be created on the 6th run on March 31st

    def logdir_setup(self, basename=None):
        self.get_logger().info(f'Using {self.loghome} as logging home')

        if basename:
            dirname = os.path.join(self.loghome, basename)
        else:
            # Use logger_time date to create directory name, e.g., 2023-03-23.002
            now = self.logger_time.strftime('%Y-%m-%d')
            count = 0
            basename = now + '.{nnn:03}'.format(nnn=count)
            dirname = os.path.join(self.loghome, basename)
            while os.path.exists(dirname):
                count = count + 1
                basename = now + '.{nnn:03}'.format(nnn=count)
                dirname = os.path.join(self.loghome, basename)

        if not os.path.exists(dirname):
            os.makedirs(dirname)

        # Point a link called 'latest' to the new directory
        # Renaming a temporary link works as 'ln -sf'
        templink = os.path.join(self.loghome, '__templn__')
        os.symlink(basename, templink)
        latest = os.path.join(self.loghome, 'latest_csv')
        os.rename(templink, latest)

        self.get_logger().info(f'New log directory: {dirname}')
        return dirname

    #           Power Controller write functions
    # PC header section

    def write_pc_header(self):
        self.pc_header = """ \
PC RPM, PC Bus Voltage (V), PC Winding Curr (A), PC Battery Curr (A), PC Status, \
PC Load Dump Current (A), PC Target Voltage (V), PC Target Curr (A), \
PC Diff PSI, PC RPM Std Dev, PC Scale, PC Retract, PC Aux Torque (mV), \
PC Bias Curr (A), PC Charge Curr (A), PC Draw Curr (A), """
        self.logfile.write(self.pc_header)

    # PC record section
    # Just write the commas unless data is the correct type

    def write_pc(self, data):
        if (type(data) is PCRecord):
            self.logfile.write(f'{data.rpm:.1f}, {data.voltage:.1f}, {data.wcurrent:.2f}, '
                               + f'{data.bcurrent:.2f}, {data.status}, {data.loaddc:.2f}, '
                               + f'{data.target_v:.1f}, {data.target_a:.2f}, '
                               + f'{data.diff_press:.3f}, {data.sd_rpm:.1f}, {data.scale:.2f}, '
                               + f'{data.retract:.2f}, {data.torque:.2f}, '
                               + f'{data.bias_current:.2f}, '
                               + f'{data.charge_curr_limit:.2f}, {data.draw_curr_limit:.2f}, ')
        else:
            self.logfile.write(',' * self.pc_header.count(','))

    #            Battery Controller write functions
    # BC header section
    def write_bc_header(self):
        self.bc_header = """ \
BC Voltage, BC Ips, BC V Balance, BC V Stopcharge, BC Ground Fault, \
BC_Hydrogen, BC Status, """
        self.logfile.write(self.bc_header)

    # BC record section
    # Just write the commas unless data is the correct type
    def write_bc(self, data):
        if (type(data) is BCRecord):
            self.logfile.write(f'{data.voltage:.1f}, {data.ips:.2f}, '
                               + f'{data.vbalance:.2f}, {data.vstopcharge:.2f}, '
                               + f'{data.gfault:.2f}, {data.hydrogen:.2f}, {data.status}, ')
        else:
            self.logfile.write(',' * self.pc_header.count(','))

    #            Crossbow AHRS Controller write functions
    # XB header section
    def write_xb_header(self):
        self.xb_header = """ \
XB Roll XB Angle (deg), XB Pitch Angle (deg), XB Yaw Angle (deg), \
XB X Rate, XB Y Rate, XB Z Rate, XB X Accel, XB Y Accel, XB Z Accel, \
XB North Vel, XB East Vel, XB Down Vel, XB Lat, XB Long, XB Alt, XB Temp, """
        self.logfile.write(self.xb_header)

    # XB record section
    # Just write the commas unless data is the correct type
    def write_xb(self, data):
        if (type(data) is XBRecord):
            imu = data.imu
            gps = data.gps
            ned = data.ned_velocity
            tmp = data.x_rate_temp
            # get roll, pitch, and yaw in degrees
            (roll, pitch, yaw) = euler_from_quaternion([imu.orientation.x,
                                                        imu.orientation.y,
                                                        imu.orientation.z,
                                                        imu.orientation.w])
            self.logfile.write(
                f'{math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, {math.degrees(yaw):.3f}, '
                + f'{imu.angular_velocity.x:.3f}, {imu.angular_velocity.y:.3f}, '
                + f'{imu.angular_velocity.z:.3f}, '
                + f'{imu.linear_acceleration.x:.3f}, {imu.linear_acceleration.y:.3f}, '
                + f'{imu.linear_acceleration.z:.3f}, '
                + f'{ned.z:.3f}, {ned.y:.3f}, {ned.z:.3f}, '
                + f'{gps.latitude:.5f}, {gps.longitude:.5f}, {gps.altitude:.3f}, '
                + f'{tmp.temperature:.3f}, ')
        else:
            self.logfile.write(',' * self.pc_header.count(','))

    #            Spring Controller write functions
    # SC header section
    # Just write the commas unless data is the correct type
    def write_sc_header(self):
        self.sc_header = """ \
SC Load Cell (lbs), SC Range Finder (in), \
SC Upper PSI, SC Lower PSI, SC Status, CTD Time, CTD Salinity, CTD Temp, """
        self.logfile.write(self.sc_header)

    # SC record section
    def write_sc(self, data):
        if (type(data) is SCRecord):
            self.logfile.write(f'{data.load_cell}, {data.range_finder:.2f}, '
                               + f'{data.upper_psi:.2f}, {data.lower_psi:.2f}, '
                               + f'{data.status}, {data.epoch}, '
                               + f'{data.salinity:.6f}, {data.temperature:.3f}, ')
        else:
            self.logfile.write(',' * self.pc_header.count(','))

    #            Trefoil Controller write functions
    # TF header section

    def write_tf_header(self):
        self.tf_header = """ \
TF Power-Loss Timeouts, TF Tether Volt, TF Batt Volt, TF Pressure psi, \
TF Qtn 1, TF Qtn 2, TF Qtn 3, TF Qtn 4, TF Mag 1 gauss, TF Mag 2, TF Mag 3, TF Status, \
TF Ang Rate 1 rad/sec, TF Ang Rate 2,  TF Ang Rate 3, TF VPE status, \
TF Accel 1 m/sec^2, TF Accel 2, TF Accel 3, TF Comms-Loss Timeouts, \
TF Maxon status, TF Motor curren mA, TF Encoder counts, """
        self.logfile.write(self.tf_header)

    # TF record section
    # Just write the commas unless data is the correct type

    def write_tf(self, data):
        if (type(data) is TFRecord):
            imu = data.imu
            mag = data.mag
            self.logfile.write(f'{data.power_timeouts}, {data.tether_voltage:.3f}, '
                               + f'{data.battery_voltage:.3f}, {data.pressure:.3f}, '
                               + f'{imu.orientation.x:.3f}, {imu.orientation.y:.3f}, '
                               + f'{imu.orientation.z:.3f}, {imu.orientation.w:.3f}, '
                               + f'{mag.magnetic_field.x:.3f}, {mag.magnetic_field.y:.3f}, '
                               + f'{mag.magnetic_field.z:.3f}, {data.status}, '
                               + f'{imu.angular_velocity.x:.3f}, '
                               + f'{imu.angular_velocity.y:.3f}, '
                               + f'{imu.angular_velocity.z:.3f}, '
                               + f'{data.vpe_status}, '
                               + f'{imu.linear_acceleration.x:.3f}, '
                               + f'{imu.linear_acceleration.y:.3f}, '
                               + f'{imu.linear_acceleration.z:.3f}, '
                               + f'{data.comms_timeouts}, {data.motor_status}, '
                               + f'{data.motor_current}, {data.encoder}, ')
        else:
            self.logfile.write(',' * self.tf_header.count(','))

    def ahrs_callback(self, data):
        self.write_record(CrossbowID, data)

    def battery_callback(self, data):
        self.write_record(BatteryConID, data)

    def spring_callback(self, data):
        self.write_record(SpringConID, data)

    def power_callback(self, data):
        self.write_record(PowerConID, data)

    def trefoil_callback(self, data):
        self.write_record(TrefoilConID, data)

    def set_params(self):
        self.declare_parameter('logfileinterval_mins', int(self.logfileinterval_sec / 60))
        self.logfileinterval_sec = \
            60 * self.get_parameter('logfileinterval_mins').get_parameter_value().integer_value


def main():
    import argparse
    parser = argparse.ArgumentParser()
    loghome_arg = parser.add_argument('--loghome', default='~/.pblogs', help='root log directory')
    logdir_arg = parser.add_argument('--logdir', help='specific log directory in loghome')
    args, extras = parser.parse_known_args()

    rclpy.init(args=extras)
    pblog = WECLogger(args.loghome if args.loghome else loghome_arg.default,
                      args.logdir if args.logdir else logdir_arg.default)
    rclpy.spin(pblog)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
