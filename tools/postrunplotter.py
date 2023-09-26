import pandas as pd
import matplotlib.pyplot as plt
import rosbag
import os
from tsplotter import *
import numpy as np

def read_out(bag_path):

    data = {'time': [], 'b_w': [], 'b_a': [], 'p_wi': [], 'v_wi': [], 'q_wi': []}

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/full_state_out_topic']):
            time = msg.header.stamp.to_sec()
            data['time'].append(time)
            data['b_w'].append(msg.b_w)
            data['b_a'].append(msg.b_a)
            data['p_wi'].append(msg.p_wi)
            data['v_wi'].append(msg.v_wi)
            data['q_wi'].append(msg.q_wi)

    df = pd.DataFrame(data)
    return df

def final_states(df):
    rot = df['q_wi'].iloc[-1]
    b_a = df['b_a'].iloc[-1]
    b_w = df['b_w'].iloc[-1]

    print('Final Mars States:')
    print(f'position1_orientation: {[rot.x,rot.y,rot.z,rot.w]}')
    print(f'core_init_bw: {[b_w.x,b_w.y,b_w.z]}')
    print(f'core_init_ba: {[b_a.x,b_a.y,b_a.z]}')

def calculate_opti_error(opti_in, mars_out, verbose = False):
    opti_in['time_3'] = opti_in['time'].round(3)
    mars_out['time_3'] = mars_out['time'].round(3)

    # Merge DataFrames on the 'time' column
    merged_data = pd.merge(opti_in, mars_out, left_on='time_3', right_on='time_3', suffixes=('_nav', '_opti'), how='inner')

    print('Mars error on Opti Data:')
    print(f'Error calculating on {merged_data.shape[0]} rows')

    dif_x = merged_data['p_wi'].apply(lambda x: x.x) - merged_data['x']
    dif_y = merged_data['p_wi'].apply(lambda x: x.y) - merged_data['y']
    dif_z = merged_data['p_wi'].apply(lambda x: x.z) - merged_data['z']

    # Calculate RMSE
    rmse_x = np.sqrt(np.mean((dif_x)**2))
    rmse_y = np.sqrt(np.mean((dif_y)**2))
    rmse_z = np.sqrt(np.mean((dif_z)**2))

    if verbose:
        print(f'RMSE (X,Y,Z): {rmse_x},{rmse_y},{rmse_z}')

    # Calculate RMSE for distance
    rmse_distance = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)
    print(f'RMSE (Distance): {(rmse_distance * 1e3).round(3)}mm')

def calculate_ts_error(ts_in, mars_out, verbose = False):
    ts_in['time_3'] = ts_in['time'].round(3)
    mars_out['time_3'] = mars_out['time'].round(3)

    # Merge DataFrames on the 'time' column
    merged_data = pd.merge(ts_in, mars_out, left_on='time_3', right_on='time_3', suffixes=('_nav', '_opti'), how='inner')
    print('Mars error on TS Data:')
    print(f'Error calculating on {merged_data.shape[0]} rows')

    dif_x = merged_data['p_wi'].apply(lambda x: x.x) - merged_data['x']
    dif_y = merged_data['p_wi'].apply(lambda x: x.y) - merged_data['y']
    dif_z = merged_data['p_wi'].apply(lambda x: x.z) - merged_data['z']

    # Calculate RMSE
    rmse_x = np.sqrt(np.mean((dif_x)**2))
    rmse_y = np.sqrt(np.mean((dif_y)**2))
    rmse_z = np.sqrt(np.mean((dif_z)**2))

    if verbose:
        print(f'RMSE (X,Y,Z): {rmse_x},{rmse_y},{rmse_z}')

    # Calculate RMSE for distance
    rmse_distance = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)
    print(f'RMSE (Distance): {(rmse_distance * 1e3).round(3)}mm')

def plot(opti_in, ts_in, mars_out):

    # Create Plots
    fig, axs = plt.subplots(2, 2, sharex=True)

    # Plot Mars and Opti Positions
    axs[0][0].plot(opti_in['time'], opti_in['x'], label='Opti x')
    axs[0][0].plot(opti_in['time'], opti_in['y'], label='Opti y')
    axs[0][0].plot(opti_in['time'], opti_in['z'], label='Opti z')

    axs[0][0].plot(ts_in['time'], ts_in['x'], label='Ts x')
    axs[0][0].plot(ts_in['time'], ts_in['y'], label='Ts y')
    axs[0][0].plot(ts_in['time'], ts_in['z'], label='Ts z')

    axs[0][0].plot(mars_out['time'], mars_out['p_wi'].apply(lambda x: x.x), label='Mars x')
    axs[0][0].plot(mars_out['time'], mars_out['p_wi'].apply(lambda x: x.y), label='Mars y')
    axs[0][0].plot(mars_out['time'], mars_out['p_wi'].apply(lambda x: x.z), label='Mars z')
    axs[0][0].set_ylabel('Position (m)')
    axs[0][0].legend()

    # Plot Mars and Opti Gyro
    axs[1][0].plot(opti_in['time'], opti_in['qx'], label='Opti qX')
    axs[1][0].plot(opti_in['time'], opti_in['qy'], label='Opti qY')
    axs[1][0].plot(opti_in['time'], opti_in['qz'], label='Opti qZ')
    axs[1][0].plot(opti_in['time'], opti_in['qw'], label='Opti qW')

    axs[1][0].plot(mars_out['time'], mars_out['q_wi'].apply(lambda x: x.x), label='Mars qX')
    axs[1][0].plot(mars_out['time'], mars_out['q_wi'].apply(lambda x: x.y), label='Mars qY')
    axs[1][0].plot(mars_out['time'], mars_out['q_wi'].apply(lambda x: x.z), label='Mars qZ')
    axs[1][0].plot(mars_out['time'], mars_out['q_wi'].apply(lambda x: x.w), label='Mars qW')
    axs[1][0].set_ylabel('Orientation')
    axs[1][0].legend()

    # Plot Mars Bias
    axs[0][1].plot(mars_out['time'], mars_out['b_w'].apply(lambda x: x.x), label='Mars W Bias X')
    axs[0][1].plot(mars_out['time'], mars_out['b_w'].apply(lambda x: x.y), label='Mars W Bias Y')
    axs[0][1].plot(mars_out['time'], mars_out['b_w'].apply(lambda x: x.z), label='Mars W Bias Z')
    axs[0][1].set_ylabel('Bias W')
    axs[0][1].legend()


    axs[1][1].plot(mars_out['time'], mars_out['b_a'].apply(lambda x: x.x), label='Mars A Bias X')
    axs[1][1].plot(mars_out['time'], mars_out['b_a'].apply(lambda x: x.y), label='Mars A Bias Y')
    axs[1][1].plot(mars_out['time'], mars_out['b_a'].apply(lambda x: x.z), label='Mars A Bias Z')
    axs[1][1].set_xlabel('Time (s)')
    axs[1][1].set_ylabel('Bias A')
    axs[1][1].legend()
    # Show the plots
    plt.show()

def main():

    folder = 'src/mars_ros/tools/data/ts_2m_box_no_rot_test_05_20230920_143644'
    bag_name = '2mRotOut.bag'

    ts_in, opti_in, _ = get_files(folder)

    opti_in = transform_opti(opti_in)

    z_rot = calculate_ts_rotation(ts_in,opti_in)
    ts_rot = R.from_euler('zyx', [z_rot, 180, 0], degrees=True)
    ts_in = transform_ts(ts_in, new_rotation_matrix = ts_rot.as_matrix())

    ts_translation = calculate_ts_translation(ts_in,opti_in)
    ts_in = transform_ts(ts_in, translation = ts_translation)

    ts_offset = calculate_ts_offset(ts_in, opti_in)
    ts_in = transform_ts(ts_in, time = ts_offset)

    mars_out = read_out(os.path.join(folder, bag_name))
    final_states(mars_out)
    calculate_ts_error(ts_in, mars_out)
    calculate_opti_error(opti_in, mars_out)

    plot(opti_in,ts_in,mars_out)

if __name__ == '__main__':
    main()