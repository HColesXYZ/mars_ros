from trajectopy_core.evaluation.comparison import compare_trajectories_absolute
from trajectopy_core.alignment.actions import align_trajectories
from trajectopy_core.settings.alignment_settings import AlignmentSettings
from trajectopy_core.settings.matching_settings import MatchingMethod, MatchingSettings
from trajectopy_core.trajectory import Trajectory
from trajectopy_core.util.spatialsorter import Sorting
from matplotlib import pyplot as plt
from trajectopy_core.plotting.trajectory_plot import plot_trajectories
from trajectopy_core.plotting.deviation_plot import plot_combined_devs
from trajectopy_core.settings.core import Settings
from postrunplotter import *

folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-with-rot_2min_09-10-23'
#folder_path = 'src/mars_ros/tools/data/box_09-10-23/box-no-rot_2min_09-10-23'

def rotate_sensor(df, rotation=[0, 0, 0]):

    df_rots = R.from_quat(df[['qx', 'qy', 'qz', 'qw']])

    rot = R.from_euler('xyz', rotation, degrees=True)

    df_rots = rot * df_rots

    df[['qx', 'qy', 'qz', 'qw']] = df_rots.as_quat()

    return df

def df_to_traj(df, name):
    xyz = df[['x', 'y', 'z']].values
    quat = df[['qx', 'qy', 'qz', 'qw']].values
    tstamps = df['time'].values
    traj = Trajectory.from_numpy(xyz,quat,tstamps)
    traj.name = name

    tstamp_index = np.unique(traj.tstamps, return_index=True)[1]
    traj.apply_index(tstamp_index) 

    return traj


#gt_traj = Trajectory.from_file(f'{folder_path}/opti.traj')
#est_traj = Trajectory.from_file(f'{folder_path}/mars.traj')


mars = read_out(os.path.join(folder_path, 'Output.bag'))
#add_vector(mars, offset_vector = np.array([0.0439, 0.0224, 0.01365]))

ts, opti, _ = get_files(folder_path)
opti = cam_to_ned(opti)
mars = mars_to_ned(mars)

mars2 = quaternions_to_euler_angles(mars)
##print(f"Mars:\n{mars2[['ex', 'ey', 'ez']].head(300)}")
opti = rotate_sensor(opti, [0, 0, 360 - 129.9751])
opti = rotate_sensor(opti, [0, 5, 0])
#opti = rotate_sensor(opti, [0, 0, 100])

#exit(1)
mars_traj = df_to_traj(mars, 'mars')
opti_traj = df_to_traj(opti, 'opti')

mars_traj, opti_traj  = mars_traj.same_sampling(opti_traj)
print(mars_traj)

params = {
        "estimation_of": {
            "helmert": True,
            #"leverarm": False,
            #"time_shift": True,
            'scale': False,
            #'sensor_rotation': True
    }
}

alignment = align_trajectories(
    traj_from=mars_traj,
    traj_to=opti_traj,
    alignment_settings=AlignmentSettings.from_config_dict(params),
    matching_settings=MatchingSettings(method=MatchingMethod.INTERPOLATION),
)
#print(alignment)
print(alignment.position_parameters)
print(alignment.rotation_parameters)
print(alignment.estimation_of)

mars_traj.apply_alignment(alignment)


plot_trajectories([mars_traj, opti_traj], dim=3)
plt.show()

exit(1)



est_traj, gt_traj  = est_traj.same_sampling(gt_traj)

print(gt_traj)
print(est_traj)

print(est_traj.rot.as_euler(seq='xyz', degrees=True))
print(est_traj.rot.as_quat())
#

ate_result = compare_trajectories_absolute(traj_ref=gt_traj, traj_test=est_traj_aligned)
plot_combined_devs(ate_result)
plt.show()
print(ate_result)