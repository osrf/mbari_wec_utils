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

import os
import gzip
import math
from datetime import datetime
from tf_transformations import euler_from_quaternion

from buoy_api import Interface
from buoy_interfaces.msg import PCRecord
from buoy_interfaces.msg import BCRecord
from buoy_interfaces.msg import XBRecord
from buoy_interfaces.msg import SCRecord
from buoy_interfaces.msg import TFRecord
import rclpy

WECLOGHOME = 'WEC_LOG_DIR'
ALTLOGHOME = '/tmp'        # assume we always have write permission to /tmp
NEWFILEINTERVAL = 2 #60*60    # limit the 'size' of each log file to one hour

# Controller IDs used as the first value in each record
BatteryConID = 0
SpringConID  = 1
PowerConID   = 2
TrefoilConID = 4
CrossbowID   = 5

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
#
class WECLogger(Interface):

    def __init__(self):
        super().__init__('sim_pblog')
        #self.set_params()
        self.logdir      = None
        self.logfile     = None
        self.logfilename = None
        self.logtime     = None
        self.pc_header   = ""
        self.bc_header   = ""
        self.xb_header   = ""
        self.sc_header   = ""
        self.tf_header   = ""
        self.logfile_setup()

    def __del__(self):
        self.close_zip_logfile()


    # Create and open a new log file
    # The system time is used to create a unique CSV log file name
    # Example: "2023.03.31T23-59-59.csv" would be created just before midnight on March 31st
    def logfile_setup(self):
        # Create new log folder at the start of this instance of the logger
        if (self.logdir == None):
            self.logdir = self.logdir_setup()

        # close existing log file and zip it shut
        if (self.logfile != None):
            self.close_zip_logfile()


        # Open new file in logdir using the date & time (2023.03.23T13.09.54.csv)
        self.logfilename = self.logdir + '/' + \
                           datetime.now().strftime("%Y.%m.%dT%I.%M.%S") + '.csv'
        self.logfile = open(self.logfilename, mode="w", encoding='utf-8')

        # Point a link called 'latest' to the new directory
        # Renaming a temporary link works as 'ln -sf'
        templink = self.logdir + '/' + '__bogus__'
        os.symlink(self.logfilename, templink)
        latest = self.logdir + '/' + 'latest'
        os.rename(templink, latest)

        self.get_logger().info(f'New log file: {self.logfilename}')
        self.write_header()


    # Write CSV header line by writing each controller section
    def write_header(self):
        # Preamble/Header section column names first
        self.logfile.write("Source ID, Timestamp (epoch seconds), ")

        # Write each header section
        self.write_pc_header()
        self.write_bc_header()
        self.write_xb_header()
        self.write_sc_header()
        self.write_tf_header()
        self.write_eol()


    # Write a complete record line by writing each controller section
    def write_record(self, source_id, data):
        # Calculate float value epoch seconds and write the record preamble/header
        nanos = (data.header.stamp.sec * 1e9) + data.header.stamp.nanosec
        timestamp = (1.0*nanos) / 1e9    # convert from nanoseconds to seconds
        self.logfile.write(f'{source_id}, {timestamp:.3f}, ')

        # Pass the data to writers, let them decide what to write
        self.write_pc(data)
        self.write_bc(data)
        self.write_xb(data)
        self.write_sc(data)
        self.write_tf(data)
        self.write_eol()

        # Create a fresh logfile when interval time has passed
        if (self.logtime == None):
            self.logtime = timestamp
        elif (timestamp > self.logtime + NEWFILEINTERVAL):
            self.logfile_setup()
            self.logtime = timestamp


    def write_eol(self):
        self.logfile.write('\n')


    def close_zip_logfile(self):
        if (self.logfile != None):
            self.logfile.close()
            with open(self.logfilename, 'rb') as logf:
                with gzip.open(f'{self.logfilename}.gz', 'wb') as gzfile:
                    self.get_logger().info(f'{self.logfilename} -> {self.logfilename}.gz')
                    gzfile.writelines(logf)
                    os.remove(self.logfilename)


    # Create a new directory for log files created for this instance of logger
    # The system date is used to create a unique directory name for this run
    # Example: "2023-03-31.005 would be created on the 6th run on March 31st
    def logdir_setup(self):
        # Use WEC_LOG_DIR env variable for base of log tree
        wec_log_dir = os.getenv(WECLOGHOME)
        if (None == wec_log_dir):
            self.get_logger().info(f'WEC_LOG_DIR env variable not set, using {ALTLOGHOME} as log home')
            wec_log_dir = ALTLOGHOME
        elif (False == os.access(wec_log_dir, os.W_OK)):
            self.get_logger().info(f'No write access to {wec_log_dir}, using {ALTLOGHOME} as log home')
            wec_log_dir = ALTLOGHOME


        # Use today's date to create directory name, e.g., 2023-03-23.002
        now = datetime.today().strftime('%Y-%m-%d')
        count = 0
        dirname = wec_log_dir + '/' + now + ".{nnn:03}".format(nnn=count)
        while (os.path.exists(dirname)):
            count = count +1
            dirname = wec_log_dir + '/' + now + ".{nnn:03}".format(nnn=count)
        
        if (False == os.path.exists(dirname)):
            os.makedirs(dirname)

        # Point a link called 'latest' to the new directory
        # Renaming a temporary link works as 'ln -sf'
        templink = wec_log_dir + '/' + '__bogus__'
        os.symlink(dirname, templink)
        latest = wec_log_dir + '/' + 'latest'
        os.rename(templink, latest)

        self.get_logger().info(f'New log directory: {dirname}')
        return dirname
        


    ############ Power Controller write functions ############
    # PC header section
    def write_pc_header(self):
        self.pc_header = """\
PC RPM, PC Bus Voltage (V), PC Winding Curr (A), PC Battery Curr (A), PC Status, \
PC Load Dump Current (A), PC Target Voltage (V), PC Target Curr (A), \
PC Diff PSI, PC RPM Std Dev, PC Scale, PC Retract, PC Aux Torque (mV), \
PC Bias Curr (A), PC Charge Curr (A), PC Draw Curr (A), \
"""
        self.logfile.write(self.pc_header)

    # PC record section
    # Just write the commas unless data is the correct type
    def write_pc(self, data):
        if (type(data) is PCRecord):
            self.logfile.write(f'{data.rpm:.1f}, {data.voltage:.1f}, {data.wcurrent:.2f}, ' +
            f'{data.bcurrent:.2f}, {data.status}, {data.loaddc:.2f}, {data.target_v:.1f}, ' +
            f'{data.target_a:.2f}, {data.diff_press:.3f}, {data.sd_rpm:.1f}, {data.scale:.2f}, ' +
            f'{data.retract:.2f}, {data.torque:.2f}, {data.bias_current:.2f}, ' +
            f'{data.charge_curr_limit:.2f}, {data.draw_curr_limit:.2f}, ')
        else:
            self.logfile.write("," * self.pc_header.count(','))


    ############ Battery Controller write functions #############
    # BC header section
    def write_bc_header(self):
        self.bc_header = """\
BC Voltage, BC Ips, BC V Balance, BC V Stopcharge, BC Ground Fault, \
BC_Hydrogen, BC Status, \
"""
        self.logfile.write(self.bc_header)

    # BC record section
    # Just write the commas unless data is the correct type
    def write_bc(self, data):
        if (type(data) is BCRecord):
            self.logfile.write(f'{data.voltage:.1f}, {data.ips:.2f}, ' +
            f'{data.vbalance:.2f}, {data.vstopcharge:.2f}, {data.gfault:.2f}, ' +
            f'{data.hydrogen:.2f}, {data.status}, ')
        else:
            self.logfile.write("," * self.bc_header.count(','))


    ############ Crossbow AHRS write functions #############
    # XB header section
    def write_xb_header(self):
        self.xb_header = """\
XB Roll XB Angle (deg), XB Pitch Angle (deg), XB Yaw Angle (deg), \
XB X Rate, XB Y Rate, XB Z Rate, XB X Accel, XB Y Accel, XB Z Accel, \
XB North Vel, XB East Vel, XB Down Vel, XB Lat, XB Long, XB Alt, XB Temp, \
"""
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
            f'{math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, {math.degrees(yaw):.3f}, ' +
            f'{imu.angular_velocity.x:.3f}, {imu.angular_velocity.y:.3f}, {imu.angular_velocity.z:.3f}, ' +
            f'{imu.linear_acceleration.x:.3f}, {imu.linear_acceleration.y:.3f}, {imu.linear_acceleration.z:.3f}, ' +
            f'{ned.z:.3f}, {ned.y:.3f}, {ned.z:.3f}, ' +
            f'{gps.latitude:.5f}, {gps.longitude:.5f}, {gps.altitude:.3f}, ' +
            f'{tmp.temperature:.3f}, ')
        else:
            self.logfile.write("," * self.xb_header.count(','))


    ############ Spring Controller write functions #############
    # SC header section
    # Just write the commas unless data is the correct type
    def write_sc_header(self):
        self.sc_header = """\
SC Load Cell (lbs), SC Range Finder (in), \
SC Upper PSI, SC Lower PSI, SC Status, CTD Time, CTD Salinity, CTD Temp, \
"""
        self.logfile.write(self.sc_header)

    # SC record section
    def write_sc(self, data):
        if (type(data) is SCRecord):
            self.logfile.write(f'{data.load_cell}, {data.range_finder:.2f}, ' +
            f'{data.upper_psi:.2f}, {data.lower_psi:.2f}, {data.status}, {data.epoch}, ' +
            f'{data.salinity:.6f}, {data.temperature:.3f}, ')
        else:
            self.logfile.write("," * self.sc_header.count(','))


    ############ Trefoil Controller write functions #############
    # TF header section
    def write_tf_header(self):
        self.tf_header = """\
TF Power-Loss Timeouts, TF Tether Volt, TF Batt Volt, TF Pressure psi, \
TF Qtn 1, TF Qtn 2, TF Qtn 3, TF Qtn 4, TF Mag 1 gauss, TF Mag 2, TF Mag 3, TF Status, \
TF Ang Rate 1 rad/sec, TF Ang Rate 2,  TF Ang Rate 3, TF VPE status, \
TF Accel 1 m/sec^2, TF Accel 2, TF Accel 3, TF Comms-Loss Timeouts, \
TF Maxon status, TF Motor curren mA, TF Encoder counts, \
"""
        self.logfile.write(self.tf_header)

    # TF record section
    # Just write the commas unless data is the correct type
    def write_tf(self, data):
        if (type(data) is TFRecord):
            imu = data.imu
            mag = data.mag
            self.logfile.write(f'{data.power_timeouts}, {data.tether_voltage:.3f}, ' +
            f'{data.battery_voltage:.3f}, {data.pressure:.3f}, ' +
            f'{imu.orientation.x:.3f}, {imu.orientation.y:.3f}, {imu.orientation.z:.3f}, {imu.orientation.w:.3f}, ' +
            f'{mag.magnetic_field.x:.3f}, {mag.magnetic_field.y:.3f}, {mag.magnetic_field.z:.3f}, {data.status}, ' +
            f'{imu.linear_acceleration.x:.3f}, {imu.linear_acceleration.y:.3f}, {imu.linear_acceleration.z:.3f}, ' +
            f'{data.vpe_status}, ' +
            f'{imu.linear_acceleration.x:.3f}, {imu.linear_acceleration.y:.3f}, {imu.linear_acceleration.z:.3f}, ' +
            f'{data.comms_timeouts}, {data.motor_status}, {data.motor_current}, {data.encoder}, ')
        else:
            self.logfile.write("," * self.tf_header.count(','))




    # Delete any unused callback
    def ahrs_callback(self, data):
        '''Callback for '/ahrs_data' topic from XBowAHRS'''
        self.write_record(CrossbowID, data)

    def battery_callback(self, data):
        '''Callback for '/battery_data' topic from Battery Controller'''
        self.write_record(BatteryConID, data)

    def spring_callback(self, data):
        '''Callback for '/spring_data' topic from Spring Controller'''
        self.write_record(SpringConID, data)

    def power_callback(self, data):
        '''Callback for '/power_data' topic from Power Controller'''
        self.write_record(PowerConID, data)
        #self.csv_log.writerow([data.seq_num, data.voltage, data.sd_rpm])

    def trefoil_callback(self, data):
        '''Callback for '/trefoil_data' topic from Trefoil Controller'''
        self.write_record(TrefoilConID, data)

    def powerbuoy_callback(self, data):
        '''Callback for '/powerbuoy_data' topic -- Aggregated data from all topics'''
        #self.get_logger().info('Received Powerbuoy data')

    def set_params(self):
        '''Use ROS2 declare_parameter and get_parameter to set policy params'''
        pass


def main():
    rclpy.init()
    pblog = WECLogger()
    rclpy.spin(pblog)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
