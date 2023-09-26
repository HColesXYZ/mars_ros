import os
import re
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial import distance

def read_data(folder,file):
    df = pd.read_csv(os.path.join(folder,file))
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
    df['time'] = (df['Timestamp(Epoch ms)']) / 1e3
    df = df.drop(columns=['PointID'])
    df = df.rename(columns={'Northing(m)': 'x', 'Easting(m)': 'y', 'Height(m)': 'z'})

    return df 

def transform_opti(df, time=36.996168639105896, new_rotation_matrix = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]):

    df['time'] = df['time'] + time  

    for index, row in df.iterrows():

        # Populate the message fields
        pos = R.from_matrix(new_rotation_matrix).apply([row['x'], row['y'], row['z']])

        df.at[index, 'x'] = pos[0]
        df.at[index, 'y'] = pos[1]
        df.at[index, 'z'] = pos[2]

        old_matrix = R.from_quat([row['qx'], row['qy'], row['qz'], row['qw']]).as_matrix()
        final_rotation_matrix = np.dot(old_matrix, new_rotation_matrix)
        # Convert back to quaternion
        new_quaternion = R.from_matrix(final_rotation_matrix).as_quat()

        df.at[index, 'qx'] = new_quaternion[0]
        df.at[index, 'qy'] = new_quaternion[1]
        df.at[index, 'qz'] = new_quaternion[2]
        df.at[index, 'qw'] = new_quaternion[3]

    return df

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

def calculate_ts_offset(ts,opti):
    id_ts_max = ts['x'].idxmax()
    id_opti_max = opti['x'].idxmax()
    time_offset = opti.loc[id_opti_max, 'time'] - ts.loc[id_ts_max, 'time']
    print(f'Total Station, OptiTrack offset: {time_offset}s')
    return time_offset

def calculate_ts_rotation(ts, opti, y_flip=True):

    y_rot = 0
    if y_flip:
        y_rot = 180

    def alignment_metric(z_rot):
        ts_rot = R.from_euler('zyx', [z_rot, y_rot, 0], degrees=True)
        aligned_ts = transform_ts(ts, 0, ts_rot.as_matrix(), [0, 0, 0])

        corners_opti = get_corners(opti)
        corners_ts = get_corners(aligned_ts)

        distances = [distance.euclidean(corners_opti[i], corners_ts[i]) for i in range(4)]
        return np.std(distances)

    def get_corners(df):
        x_max_id = df['x'].idxmax()
        x_min_id = df['x'].idxmin()
        y_max_id = df['y'].idxmax()
        y_min_id = df['y'].idxmin()

        return [
            [df.loc[x_max_id, 'x'], df.loc[x_max_id, 'y']],
            [df.loc[x_min_id, 'x'], df.loc[x_min_id, 'y']],
            [df.loc[y_max_id, 'x'], df.loc[y_max_id, 'y']],
            [df.loc[y_min_id, 'x'], df.loc[y_min_id, 'y']]
        ]

    result = minimize_scalar(alignment_metric, bounds=(0, 360), method='bounded')
    print(f"The best z_rot value is {result.x:.3f} degrees with an alignment error of {result.fun:.4f}.")
    return result.x

def calculate_ts_translation(ts,opti):
    x = np.max(opti['x']) - np.max(ts['x'])
    y = np.max(opti['y']) - np.max(ts['y'])
    z = np.max(opti['z']) - np.max(ts['z'])

    print(f"The best translation values (x,y,z) are {x:.3f}, {y:.3f}, {z:.3f}")
    return [x,y,z]

def initial_conditions(df):
    print('Initial Conditions')
    pos = [df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0]]
    print(f'Position XYZ: {pos}')
    rot = [df['qx'].iloc[0], df['qy'].iloc[0], df['qz'].iloc[0], df['qw'].iloc[0]]
    print(f'Orientation XYZW: {rot}')
   
def get_files(folder_path):

    # Define regex patterns for each type of file
    ts_pattern = r'^ts_.*\.csv$'
    opti_pattern = r'^[^a-zA-Z]+\.csv$'
    imu_pattern = r'.*updated\.csv$'

    # Function to filter files based on regex pattern
    def filter_files(pattern, files):
        return [file for file in files if re.match(pattern, file)][0]

    # Get the list of files in the folder
    files = os.listdir(folder_path)

    # Find the appropriate files
    ts_file = filter_files(ts_pattern, files)
    opti_file = filter_files(opti_pattern, files)
    imu_file = filter_files(imu_pattern, files)

    # Print the results
    print('Files found:')
    print("TS File:", ts_file)
    print("Opti File:", opti_file)
    print("IMU File:", imu_file)
    print()

    return (preprocess_ts_data(read_data(folder_path,ts_file)), 
            preprocess_opti_data(read_data(folder_path,opti_file)), 
            preprocess_imu_data(read_data(folder_path,imu_file)))

def plot(ts, opti, imu):

    # Create three separate plots
    plt.figure(figsize=(10, 8))

    # Plot for opti DataFrame
    plt.subplot(4, 1, 1)
    plt.plot(opti['time'], opti['x'], label='x')
    plt.plot(opti['time'], opti['y'], label='y')
    plt.plot(opti['time'], opti['z'], label='z')
    plt.ylabel('Opti Track (m)')
    plt.xlabel('Time (s)')
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(ts['time'], ts['x'], label='x')
    plt.plot(ts['time'], ts['y'], label='y')
    plt.plot(ts['time'], ts['z'], label='z')
    plt.ylabel('Total Station (m)')
    plt.xlabel('Time (s)')
    plt.legend()

    # Plot for ts DataFrame
    plt.subplot(4, 1, 3)
    plt.plot(ts['x'], ts['y'], label='ts')
    plt.plot(opti['x'], opti['y'], label='opti')
    plt.xlabel('X Pos (m)')
    plt.ylabel('Y Pos (m)')
    plt.legend()

    # Plot for imu DataFrame
    plt.subplot(4, 1, 4)
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
    folder_path = 'src/mars_ros/tools/data/ts_2m_box_no_rot_test_05_20230920_143644'
    ts, opti, imu = get_files(folder_path)

    opti = transform_opti(opti)

    z_rot = calculate_ts_rotation(ts,opti)
    ts_rot = R.from_euler('zyx', [z_rot, 180, 0], degrees=True)
    ts = transform_ts(ts, new_rotation_matrix = ts_rot.as_matrix())

    ts_translation = calculate_ts_translation(ts,opti)
    ts = transform_ts(ts, translation = ts_translation)

    ts_offset = calculate_ts_offset(ts, opti)
    ts = transform_ts(ts, time = ts_offset)

    plot(ts, opti, imu)

if __name__ == '__main__':
    main()



