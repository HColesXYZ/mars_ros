from tsplotter import *


ts_opti_time_gain = -3060.77811
opti_imu_time_gain = 0
ts_rot = R.from_euler('zyx', [41.18, 0, 0], degrees=True).as_matrix()
ts_trans = [14.86188, -10.10387, -0.02135]
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23'
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_5m_09-10-23'
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_2min_09-10-23'
folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_5m_09-10-23'


def write_ts(ts):
    file_name = 'totalStation_transform.csv'
    file_path = f'{folder_path}/{file_name}'

    ts = ts[['time', 'x', 'y', 'z', 'Hz Angle', 'V Angle', 'Slope Distance', 'EDM Kind', 'EDM Mode']].copy()

    print(f"Columns in {file_name}: {', '.join(ts.columns)}")
    ts.to_csv(file_path, index=False)

def write_opti(opti):
    file_name = 'opti_transform.csv'
    file_path = f'{folder_path}/{file_name}'

    opti = opti[['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'transitLatency', 'clientLatency']].copy()

    print(f"Columns in {file_name}: {', '.join(opti.columns)}")
    opti.to_csv(file_path, index=False)

def write_imu(imu):
    file_name = 'imu_transform.csv'
    file_path = f'{folder_path}/{file_name}'

    print(f"Columns in {file_name}: {', '.join(imu.columns)}")
    imu.to_csv(file_path, index=False)

def write_conversions(ts,opti,imu):
    write_ts(ts)
    write_opti(opti)
    write_imu(imu)

def main():

    ts, opti, imu = get_files(folder_path)
    opti = transform_opti(opti, time = opti_imu_time_gain)
    imu = transform_imu(imu)
    ts = transform_ts(ts, time = ts_opti_time_gain, new_rotation_matrix=ts_rot, translation=ts_trans)

    write_conversions(ts,opti,imu)

if __name__ == '__main__':
    main()