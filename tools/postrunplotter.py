import pandas as pd
import matplotlib.pyplot as plt
import rosbag
import os
from tsplotter import *

def read_out(bag_path):
    data = {'time': [], 'b_w': [], 'b_a': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}

    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=['/full_state_out_topic']):
            time = msg.header.stamp.to_sec()
            data['time'].append(time)
            data['b_w'].append(msg.b_w)
            data['b_a'].append(msg.b_a)
            data['x'].append(msg.p_wi.x)
            data['y'].append(msg.p_wi.y)
            data['z'].append(msg.p_wi.z)
            data['qx'].append(msg.q_wi.x)
            data['qy'].append(msg.q_wi.y)
            data['qz'].append(msg.q_wi.z)
            data['qw'].append(msg.q_wi.w)

    df = pd.DataFrame(data)
    return df

def final_states(df):
    row = df[df['time'] <= df['time'].iloc[-1] - 5].iloc[-1]
    b_a = row['b_a']
    b_w = row['b_w']

    print('Final Mars States:')
    print(f'position1_orientation: {[row.qx,row.qy,row.qz,row.qw]}')
    print(f'core_init_bw: {[b_w.x,b_w.y,b_w.z]}')
    print(f'core_init_ba: {[b_a.x,b_a.y,b_a.z]}\n')

def initial_states(df):
    print('Initial OptiTrack States:')
    rot = [df['qx'].iloc[0], df['qy'].iloc[0], df['qz'].iloc[0], df['qw'].iloc[0]]
    print(f'position1_orientation: {rot}\n')

def plot_out(opti_in, ts_in, mars_out, do_ts = True):

    # Create Plots
    _, axs = plt.subplots(2, 2, sharex=True)

    # Plot Mars and Opti Positions
    axs[0][0].plot(opti_in['time'], opti_in['x'], label='Opti x')
    axs[0][0].plot(opti_in['time'], opti_in['y'], label='Opti y')
    axs[0][0].plot(opti_in['time'], opti_in['z'], label='Opti z')

    if do_ts:
        axs[0][0].plot(ts_in['time'], ts_in['x'], label='Ts x')
        axs[0][0].plot(ts_in['time'], ts_in['y'], label='Ts y')
        axs[0][0].plot(ts_in['time'], ts_in['z'], label='Ts z')

    axs[0][0].plot(mars_out['time'], mars_out['x'], label='Mars x')
    axs[0][0].plot(mars_out['time'], mars_out['y'], label='Mars y')
    axs[0][0].plot(mars_out['time'], mars_out['z'], label='Mars z')
    axs[0][0].set_ylabel('Position (m)')
    axs[0][0].legend()

    # Plot Mars and Opti Gyro
    axs[1][0].plot(opti_in['time'], opti_in['qx'], label='Opti qX')
    axs[1][0].plot(opti_in['time'], opti_in['qy'], label='Opti qY')
    axs[1][0].plot(opti_in['time'], opti_in['qz'], label='Opti qZ')
    axs[1][0].plot(opti_in['time'], opti_in['qw'], label='Opti qW')

    axs[1][0].plot(mars_out['time'], mars_out['qx'], label='Mars qX')
    axs[1][0].plot(mars_out['time'], mars_out['qy'], label='Mars qY')
    axs[1][0].plot(mars_out['time'], mars_out['qz'], label='Mars qZ')
    axs[1][0].plot(mars_out['time'], mars_out['qw'], label='Mars qW')
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

    # Create a new figure for the last plot
    _, ax2 = plt.subplots()

    if do_ts:
        ax2.plot(ts_in['x'], ts_in['y'], label='ts')
    ax2.plot(opti_in['x'], opti_in['y'], label='opti')
    ax2.plot(mars_out['x'], mars_out['y'], label='mars')
    ax2.set_xlabel('X Pos (m)')
    ax2.set_ylabel('Y Pos (m)')
    ax2.legend()

    _, ax3 = plt.subplots()

    # Plot Mars and Opti Positions
    ax3.plot(opti_in['time'], opti_in['x'], label='Opti x')
    ax3.plot(opti_in['time'], opti_in['y'], label='Opti y')
    ax3.plot(opti_in['time'], opti_in['z'], label='Opti z')

    if do_ts:
        ax3.plot(ts_in['time'], ts_in['x'], label='Ts x')
        ax3.plot(ts_in['time'], ts_in['y'], label='Ts y')
        ax3.plot(ts_in['time'], ts_in['z'], label='Ts z')

    ax3.plot(mars_out['time'], mars_out['x'], label='Mars x')
    ax3.plot(mars_out['time'], mars_out['y'], label='Mars y')
    ax3.plot(mars_out['time'], mars_out['z'], label='Mars z')
    ax3.set_ylabel('Position (m)')
    ax3.legend()

    plt.show()

def write_rpg(mars_out, opti_in, folder):
    
    # Define the columns to filter and their corresponding new names
    columns_to_filter = ['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    new_column_names = {'time': 'timestamp', 'x': 'tx', 'y': 'ty', 'z': 'tz'}
    #timestamp tx ty tz qx qy qz qw

    # Perform filtering and renaming
    #stamped_traj_estimate.txt
    traj_est = mars_out.filter(items=columns_to_filter).rename(columns=new_column_names)
    #stamped_groundtruth.txt
    groundtruth = opti_in.filter(items=columns_to_filter).rename(columns=new_column_names)
    
    print('Writing RPG Values')
    print(f"Columns in Mars: {', '.join(traj_est.columns)}")
    print(f"Columns in Opti: {', '.join(groundtruth.columns)}\n")

    traj_est.to_csv(f'{folder}/stamped_traj_estimate.txt', sep=' ', index=False)
    groundtruth.to_csv(f'{folder}/stamped_groundtruth.txt', sep=' ', index=False)
    # Return the modified dataframes

def add_vector(df, offset_vector):
    # Assuming df contains columns 'qx', 'qy', 'qz', 'qw', 'x', 'y', 'z'
    # Make sure to replace these with actual column names if they are different.

    # Extract the quaternion values
    qx = df['qx'].values
    qy = df['qy'].values
    qz = df['qz'].values
    qw = df['qw'].values

    # Create Rotation object
    rot = R.from_quat(np.column_stack((qx, qy, qz, qw)))

    # Apply rotation to the offset vector
    offset_vector_rotated = rot.apply(offset_vector)

    # Add the rotated offset vector to the 'x', 'y', 'z' columns
    df[['x', 'y', 'z']] = df[['x', 'y', 'z']] + offset_vector_rotated

    return df

def main():
    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_2min_09-10-23'
    #ts_imu_time_gain = -3060.77811
    #mars_rot = R.from_euler('zyx', [41.15411, 0, 0], degrees=True).as_matrix()
    #mars_trans = [14.86733, -10.13975, -0.03490]

    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_5m_09-10-23'
    #ts_imu_time_gain = -3060.61450
    #mars_rot = R.from_euler('zyx', [41.00077, 0, 0], degrees=True).as_matrix()
    #mars_trans = [14.85980, -10.16156, -0.04204]

    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23' # -3062.124967813492s
    #ts_imu_time_gain = -3060.77811
    #mars_rot = R.from_euler('zyx', [41.21194, 0, 0], degrees=True).as_matrix()
    #mars_trans = [14.86188, -10.10387, -0.02135]

    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_5m_09-10-23'
    #ts_imu_time_gain = -3060.77811
    #mars_rot = R.from_euler('zyx', [41.18, 0, 0], degrees=True).as_matrix()
    #mars_trans = [14.86188, -10.10387, -0.02135]

    #folder = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23' # new lever
    #ts_imu_time_gain = -3060.77811
    #mars_rot = R.from_euler('zyx', [40.99881, 0, 0], degrees=True).as_matrix()
    #mars_trans = [14.84544, -10.14781, -0.03321]

    folder_path = 'src/mars_ros/tools/data/Archive 1'

    bag_name = 'Output.bag'
    opti_imu_time_gain = 0
    
    #do_mars_transform = False

    _, opti_in, _ = get_files(folder_path, do_ts=False)
    #opti_in = transform_opti(opti_in, time = opti_imu_time_gain)
    opti_in = time_shift(cam_to_mars(opti_in), opti_imu_time_gain)
    #ts_in = transform_ts(ts_in, time = ts_imu_time_gain)

    
    mars_out = read_out(os.path.join(folder_path, bag_name))
    final_states(mars_out)

    mars_out = chop_first(mars_out, seconds = 30)

    # from figure 6-1 of STIM sheet
    #add_vector(mars_out, offset_vector = np.array([0.0439, 0.0224, 0.01365]))


    #if do_mars_transform:
    #    mars_rot, mars_trans, _ = calculate_transformation(mars_out, opti_in, do_time = False)

    #mars_out = transform_ts(mars_out, new_rotation_matrix = mars_rot, translation = mars_trans)
    #ts_in = transform_ts(ts_in, new_rotation_matrix = mars_rot, translation = mars_trans)
    

    #write_rpg(mars_out,opti_in, folder)

    #calculate_error(opti_in, mars_out)

    plot(opti_in,None,mars_out, do_ts = False)

if __name__ == '__main__':
    main()