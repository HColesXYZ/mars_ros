import pandas as pd
import re
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
import math
import os
from tsplotter import *

def downsample(df, frequency, start_time):
    # Split the DataFrame into two parts: first 5 seconds and the rest
    #df_first_5_seconds = df[df['time'] <= df['time'].iloc[0] + start_time]
    #df_rest = df[df['time'] > df['time'].iloc[0] + start_time]

    #df_combined = pd.concat([df_first_5_seconds, df_rest.iloc[::frequency]])
    df_combined = df.iloc[::frequency]
    mean_diff = df_combined['time'].diff().dropna().mean()
    print(f'Down Sampled to: {mean_diff:.3f}Hz')

    return df_combined 

def create_header_stamp(time_value):
    nsecs, secs = math.modf(time_value)
    return Header(stamp=rospy.Time(secs=int(secs), nsecs=int(nsecs * 1e9)))

def write_imu(df,bag):
    for _, row in df.iterrows():
        msg = Imu()

        # Create a header for the message
        msg.header = create_header_stamp(row['time'])

        # Populate the message fields
        msg.linear_acceleration.x = -row['ax']
        msg.linear_acceleration.y = -row['ay']
        msg.linear_acceleration.z = row['az']

        msg.angular_velocity.x = -row['gx']
        msg.angular_velocity.y = -row['gy']
        msg.angular_velocity.z = row['gz']

        bag.write('/imu_in_topic', msg, t=msg.header.stamp)

def write_opti(df,bag):
    # Loop through the DataFrame and create messages

    for _, row in df.iterrows():
        msg = PoseStamped()
        
        # Create a header for the message 
        msg.header = create_header_stamp(row['time'])

        # Populate the message fields
        msg.pose.position.x = row['x']
        msg.pose.position.y = row['y']
        msg.pose.position.z = row['z']

        #these quats arent used in mars so this does not matter
        #msg.pose.orientation.w = row['qx']
        #msg.pose.orientation.x = row['qy']
        #msg.pose.orientation.y = row['qz']
        #msg.pose.orientation.z = row['qw']

        bag.write('/pose_in_topic', msg, t=msg.header.stamp)

def write_ts(df,bag):
    # Loop through the DataFrame and create messages

    for _, row in df.iterrows():
        msg = PoseStamped()
        
        # Create a header for the message 
        msg.header = create_header_stamp(row['time'])

        # Populate the message fields
        msg.pose.position.x = row['x']
        msg.pose.position.y = row['y']
        msg.pose.position.z = row['z']

        bag.write('/pose_in_topic', msg, t=msg.header.stamp)

def main():

    folder = 'src/mars_ros/tools/data/ts_2m_box_no_rot_test_05_20230920_143644'
    bag_name = '2mRotIn.bag'
 
    bag = rosbag.Bag(os.path.join(folder, bag_name), 'w')

    ts, opti, imu = get_files(folder)

    #opti = downsample(opti, 240, 5)
    opti = transform_opti(opti)

    z_rot = calculate_ts_rotation(ts,opti)
    ts_rot = R.from_euler('zyx', [z_rot, 180, 0], degrees=True)
    ts = transform_ts(ts, new_rotation_matrix = ts_rot.as_matrix())

    ts_translation = calculate_ts_translation(ts,opti)
    ts = transform_ts(ts, translation = ts_translation)

    ts_offset = calculate_ts_offset(ts, opti)
    ts = transform_ts(ts, time = ts_offset)
    ts = downsample(ts, 8, 0)
    #opti = downsample(opti, 240, 0)

    write_imu(imu, bag)
    #write_opti(opti, bag)
    write_ts(ts, bag)
    bag.close()
    print(f"DataFrame data from '{folder}' has been packed into '{bag_name}'")


if __name__ == '__main__':
    main()