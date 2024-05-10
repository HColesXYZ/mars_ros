from postrunplotter import *


ts_opti_time_gain = -3060#.77811
opti_imu_time_gain = 0
ts_rot = R.from_euler('zyx', [-41, 0, 0], degrees=True).as_matrix()
ts_trans = [14.84544, 10.14781, 0.03321]
folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23'
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_5m_09-10-23'
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_2min_09-10-23'
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_5m_09-10-23'


def write_traj(df, name, fields):

    df_copy = df.copy().rename(columns={'time': 't', 'x': 'px', 'y': 'py', 'z': 'pz'})
    df_copy = df_copy[fields]

    header = f'#epsg 0\n#name {name}\n#nframe enu\n#fields {",".join(fields)}\n'
    file_name = f'{name}.traj'
    file_path = f'{folder_path}/{file_name}'

    with open(file_path, 'w') as file:
        file.write(header)
        df_copy.to_csv(file, index=False, header=False)

def write_conversions(ts, mars,opti):
    write_traj(ts, 'ts', ['t', 'px', 'py', 'pz'])
    write_traj(mars, 'mars', ['t', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw'])
    #write_traj(mars, 'mars', ['t', 'px', 'py', 'pz', 'ex', 'ey', 'ez'])
    write_traj(opti, 'opti', ['t', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw'])
    #write_traj(opti, 'opti', ['t', 'px', 'py', 'pz', 'ex', 'ey', 'ez'])

def main():

    mars = read_out(os.path.join(folder_path, 'Output.bag'))
    #mars_out = chop_first(mars_out, seconds = 30)

    ts, opti, _ = get_files(folder_path)
    #opti = transform_opti(opti, time = opti_imu_time_gain)
    ts = transform_ts(ts, time=ts_opti_time_gain)

    add_vector(mars, offset_vector = np.array([0.0439, 0.0224, 0.01365]))

    #opti = mars_to_ned(opti)
    print(mars[['qx', 'qy', 'qz', 'qw']].head(3))
    opti = cam_to_ned(opti)


    mars = mars_to_ned(mars)
    #mars = transform_ts(mars, new_rotation_matrix = ts_rot, translation = ts_trans)

    print(mars[['qx', 'qy', 'qz', 'qw']].head(3))
    ts = mars_to_ned(ts)

    #opti = rotate_ned(opti,0,0,180)
    mars2 = quaternions_to_euler_angles(mars)
    print(f"Mars:\n{mars2[['ex', 'ey', 'ez']].head(3)}")
    opti2 = quaternions_to_euler_angles(opti)
    print(f"Opti:\n{opti2[['ex', 'ey', 'ez']].head(3)}")
    #opti2 = adjust_heading(opti2, 0,0, -131.1958)
    #exit(1)
    write_conversions(ts, mars, opti)

if __name__ == '__main__':
    main()