import os
import re
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.optimize import minimize

def read_data(folder,file):
    df = pd.read_csv(os.path.join(folder,file), low_memory=False)
    return df

def preprocess_imu_data(df):
    match = re.search(r'\d+\.\d+', df['trigger_time'][0])
    offset = float(match.group())
    df = df.drop(index=0).reset_index(drop=True)
    df['time'] = df['trigger_time'].astype(float) + offset
    return df

def preprocess_opti_data(df):
    df.rename(columns=lambda x: x.strip(), inplace=True)
    df['time'] = (df['TimeCode']) / 1e9
    df = df.rename(columns={'positionX': 'x', 'positionY': 'y', 'ppositionZ': 'z'})
    df = df.rename(columns={'quaternionX': 'qx', 'quaternionY': 'qy', 'quaternionZ': 'qz', 'quaternionW': 'qw'})

    return df

def preprocess_ts_data(df):

    df['time'] = df['Sensortime'] / 1e3
    df = df.drop(columns=['Point ID'])
    df = df.rename(columns={'Northing': 'x', 'Easting': 'y', 'Height': 'z'})
    df['y'] = - df['y']

    df.drop_duplicates(subset=['Sensortime'])
    df.sort_values(by='Sensortime')

    time_0 = pd.to_datetime(df['Date'][0] + ' ' + df['Time'][0], format='%d.%m.%Y %H:%M:%S.%f').timestamp()

    df['time'] = df['time'] - df['time'][0]
    df['time'] = df['time'] + time_0

    return df

def transform_opti(df, time = 0):

    # takes in a dataframe and changes:
    #from: y (up), z (forward), x(left)  (right handed)
    # to: z (up), x (forward), y(left) (right handed)

    df_copy = df.copy()

    df_copy['time'] = df_copy['time'] + time  

    df_copy['x'] = df['z']
    df_copy['y'] = df['x']
    df_copy['z'] = df['y']

    df_copy['qx'] = df['qz']
    df_copy['qy'] = df['qx']
    df_copy['qz'] = df['qy']

    return df_copy

def transform_ts(df, time=0, new_rotation_matrix=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], translation=[0,0,0]):
    # Create a copy of the input DataFrame
    df_copy = df.copy()

    # Apply time offset
    df_copy['time'] = df_copy['time'] + time

    for index, row in df_copy.iterrows():
        # Populate the message fields
        pos = R.from_matrix(new_rotation_matrix).apply([row['x'], row['y'], row['z']])

        df_copy.at[index, 'x'] = pos[0] + translation[0]
        df_copy.at[index, 'y'] = pos[1] + translation[1]
        df_copy.at[index, 'z'] = pos[2] + translation[2]

    return df_copy

def transform_imu(df):

    # takes in a dataframe and changes:
    #from: z (up), x (back), y (right)  (right handed)
    # to: z (up), x (forward), y(left) (right handed)

    df_copy = df.copy()

    df_copy['ax'] = -df['ax']
    df_copy['ay'] = -df['ay']

    df_copy['gx'] = -df['gx']
    df_copy['gy'] = -df['gy']

    return df_copy

def calculate_error(opti_in, mars_out, verbose = True, round = 3):
    opti_in['time_r'] = opti_in['time'].round(round)
    mars_out['time_r'] = mars_out['time'].round(round)

    # Merge DataFrames on the 'time' column
    merged_data = pd.merge(opti_in, mars_out, left_on='time_r', right_on='time_r', suffixes=('_mars', '_opti'), how='inner')

    dif_x = merged_data['x_mars'] - merged_data['x_opti']
    dif_y = merged_data['y_mars'] - merged_data['y_opti']
    dif_z = merged_data['z_mars'] - merged_data['z_opti']

    # Calculate RMSE
    rmse_x = np.sqrt(np.mean((dif_x)**2))
    rmse_y = np.sqrt(np.mean((dif_y)**2))
    rmse_z = np.sqrt(np.mean((dif_z)**2))

    # Calculate RMSE for distance
    rmse_distance = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)
    if(verbose):
        print('Mars error on Opti Data:')
        print(f'Error calculating on {merged_data.shape[0]} rows')
        print(f'RMSE (Distance): {(rmse_distance * 1e3).round(3)}mm')
    return rmse_distance

def calculate_transformation(df1, df2, do_time = True):

    t_bound = (-3070, -3050)
    if not do_time:
        t_bound = (0,0)

    def optimization_function(params):
        t, x, y, z, z_rot = params
        rot = R.from_euler('zyx', [z_rot, 0, 0], degrees=True).as_matrix()
        trans = [x, y, z]

        aligned_df1 = transform_ts(df1, time= t, new_rotation_matrix=rot, translation=trans)

        distance = calculate_error(df2, aligned_df1, verbose=False)
        
        return distance

    print('Calculating best Rotation and Translation Values...')
    result = minimize(optimization_function, [sum(t_bound) / 2, 15, -10, 0, 41], 
                      bounds=[t_bound, (13, 16), (-12, -8), (-1, 1), (30, 50)], method='Powell')

    t, x, y, z, z_rot = result.x
    print(f"The best values for x, y, and z are [{x:.5f}, {y:.5f}, {z:.5f}] meters.")
    print(f"The best value for z_rot is {z_rot:.5f} degrees.")
    print(f"The best values for t is {t:.5f} seconds.")

    rot = R.from_euler('zyx', [z_rot, 0, 0], degrees=True).as_matrix()
    return rot, [x,y,z], t

def imu_noise(imu, stationary_period, noise_scale = 1.0, bias_scale = 1.0):
    # Extract the first and last `stationary_period` seconds of data
    first_period_data = imu[imu['time'] <= imu['time'].min() + stationary_period]
    last_period_data = imu[imu['time'] >= imu['time'].max() - stationary_period]
    stationary_data = pd.concat([first_period_data, last_period_data])
    # Calculate the noise values
    gyro_noise = stationary_data[['gx', 'gy', 'gz']].std().mean() * noise_scale
    acc_noise = stationary_data[['ax', 'ay', 'az']].std().mean() * noise_scale

    # Print the results
    print(f'gyro_rate_noise: {gyro_noise}')
    print(f'acc_noise: {acc_noise}')
    print(f'gyro_bias_noise: {gyro_noise * bias_scale}')
    print(f'acc_bias_noise: {acc_noise * bias_scale}')

    # Plot the stationary period
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(stationary_data['time'], stationary_data['gx'], label='gx')
    plt.plot(stationary_data['time'], stationary_data['gy'], label='gy')
    plt.plot(stationary_data['time'], stationary_data['gz'], label='gz')
    plt.legend()
    plt.title('Gyro Data')

    plt.subplot(2, 1, 2)
    plt.plot(stationary_data['time'], stationary_data['ax'], label='ax')
    plt.plot(stationary_data['time'], stationary_data['ay'], label='ay')
    plt.plot(stationary_data['time'], stationary_data['az'], label='az')
    plt.legend()
    plt.title('Accelerometer Data')

    plt.tight_layout()
    plt.show()

def chop_first(df, seconds = 0):
    return df[df['time'] > df['time'].iloc[0] + seconds]

def get_files(folder_path, do_ts = True):

    # Define regex patterns for each type of file
    ts_pattern = r'^t.*\.csv$'
    opti_pattern = r'^[^a-zA-Z]+\.csv$'
    imu_pattern = r'.*updated\.csv$'

    # Function to filter files based on regex pattern
    def filter_files(pattern, files):
        return [file for file in files if re.match(pattern, file)][0]

    # Get the list of files in the folder
    files = os.listdir(folder_path)

    # Find the appropriate files
    ts_data = None
    ts_file = None
    if(do_ts):
        ts_file = filter_files(ts_pattern, files)
        ts_data = preprocess_ts_data(read_data(folder_path,ts_file))
    opti_file = filter_files(opti_pattern, files)
    opti_data = preprocess_opti_data(read_data(folder_path,opti_file))
    imu_file = filter_files(imu_pattern, files)
    imu_data = preprocess_imu_data(read_data(folder_path,imu_file))

    # Print the results
    print('Files found:')
    print("TS File:", ts_file)
    print("Opti File:", opti_file)
    print("IMU File:", imu_file)
    print()

    return (ts_data, opti_data, imu_data)

def plot(ts, opti, imu):

    # Create three separate plots
    plt.figure(figsize=(10, 8))

    # Plot for opti DataFrame
    plt.subplot(3, 1, 1)
    plt.plot(opti['time'], opti['x'], label='opti x')
    plt.plot(opti['time'], opti['y'], label='opti y')
    plt.plot(opti['time'], opti['z'], label='opti z')
    plt.plot(ts['time'], ts['x'], label='ts x')
    plt.plot(ts['time'], ts['y'], label='ts y')
    plt.plot(ts['time'], ts['z'], label='ts z')
    plt.ylabel('Position (m)')
    plt.xlabel('Time (s)')
    plt.legend()

    # Plot for z plane
    plt.subplot(3, 1, 2)
    plt.plot(ts['x'], ts['y'], label='ts')
    plt.plot(opti['x'], opti['y'], label='opti')
    plt.xlabel('X Pos (m)')
    plt.ylabel('Y Pos (m)')
    plt.legend()

    # Plot for imu DataFrame
    plt.subplot(3, 1, 3)
    plt.plot(imu['time'], imu['ax'], label='ax')
    plt.plot(imu['time'], imu['ay'], label='ay')
    plt.plot(imu['time'], imu['az'], label='az')
    plt.xlabel('Time (s)')
    plt.ylabel('Accel Data (m/s^2)')
    plt.legend()

    # Adjust layout
    plt.tight_layout()

    # Show the plots
    plt.show()

def main():
    #folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_2min_09-10-23'
    #ts_opti_time_gain = -3060.77811
    #ts_rot = R.from_euler('zyx', [41.02884, 0, 0], degrees=True).as_matrix()
    #ts_trans = [14.94909, -10.31693, -0.19395]

    #folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_5m_09-10-23'
    #ts_opti_time_gain = -3060.61450
    #ts_rot = R.from_euler('zyx', [41.01596, 0, 0], degrees=True).as_matrix()
    #ts_trans = [14.97019, -10.29793, -0.20497]

    #folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23'
    #ts_opti_time_gain = -3060.77811
    #ts_rot = R.from_euler('zyx', [41.01596, 0, 0], degrees=True).as_matrix()
    #ts_trans = [14.97019, -10.29793, -0.20497]

    folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_5m_09-10-23'
    ts_opti_time_gain = -3060.77811
    ts_rot = R.from_euler('zyx', [41.01596, 0, 0], degrees=True).as_matrix()
    ts_trans = [14.97019, -10.29793, -0.20497]

    opti_imu_time_gain = 0

    ts, opti, imu = get_files(folder_path)
    opti = transform_opti(opti, time = opti_imu_time_gain)
    imu = transform_imu(imu)

    do_imu_noise = False
    do_ts_transform = False

    if(do_imu_noise):
        imu_noise(imu, stationary_period = 20, noise_scale= 1, bias_scale=0.10)

    if(do_ts_transform):
        ts_rot, ts_trans, ts_opti_time_gain = calculate_transformation(ts,opti)

    ts = transform_ts(ts, time = ts_opti_time_gain, new_rotation_matrix=ts_rot, translation=ts_trans)

    calculate_error(opti, ts, verbose=True, round=2)

    plot(ts, opti, imu)

if __name__ == '__main__':
    main()



