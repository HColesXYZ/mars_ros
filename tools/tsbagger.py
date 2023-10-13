import rospy
import rosbag
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import os
from tsplotter import *

def downsample(df):

    df['temp'] = df['time'].round(0)
    # Use drop_duplicates to keep only the first entry for each second
    df_downsampled = df.drop_duplicates(subset=['temp'], keep='first').reset_index(drop=True)
    df_downsampled = df_downsampled.drop('temp', axis=1)
    
    mean_diff = df_downsampled['time'].diff().dropna().mean()
    print(f'Down Sampled to: {mean_diff:.3f}Hz')
    
    return df_downsampled

def create_header_stamp(time_value):
    nsecs, secs = math.modf(time_value)
    return Header(stamp=rospy.Time(secs=int(secs), nsecs=int(nsecs * 1e9)))

def write_imu(df,bag):
    for _, row in df.iterrows():
        msg = Imu()

        # Create a header for the message
        msg.header = create_header_stamp(row['time'])

        # Populate the message fields
        msg.linear_acceleration.x = row['ax']
        msg.linear_acceleration.y = row['ay']
        msg.linear_acceleration.z = row['az']

        msg.angular_velocity.x = row['gx']
        msg.angular_velocity.y = row['gy']
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
        msg.pose.orientation.w = row['qx']
        msg.pose.orientation.x = row['qy']
        msg.pose.orientation.y = row['qz']
        msg.pose.orientation.z = row['qw']

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
    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23'
    #ts_opti_time_gain = -3060.77811

    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_5m_09-10-23'
    #ts_opti_time_gain = -3060.61450

    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_2min_09-10-23'
    #ts_opti_time_gain = -3060.77811

    folder = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_5m_09-10-23'
    ts_opti_time_gain = -3060.77811

    opti_imu_time_gain = 0

    ts, opti, imu = get_files(folder)
    opti = transform_opti(opti, time = opti_imu_time_gain)
    ts = transform_ts(ts, time = ts_opti_time_gain)
    imu = transform_imu(imu)
    imu = chop_first(imu, seconds = 1)

    bag_name = 'Input.bag'
    bag = rosbag.Bag(os.path.join(folder, bag_name), 'w')

    #ts = downsample(ts)
    #opti = downsample(opti)

    write_imu(imu, bag)
    #write_opti(opti, bag)
    write_ts(ts, bag)
    bag.close()
    print(f"DataFrame data from '{folder}' has been packed into '{bag_name}'")


if __name__ == '__main__':
    main()