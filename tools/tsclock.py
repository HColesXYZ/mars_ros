import numpy as np
from scipy.spatial.transform import Rotation as R
from tsplotter import *
import math

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial import distance
from scipy.spatial.transform import Rotation as R

def calculate_ts_rotation(ts, opti, y_flip=True):

    def alignment_metric(z_rot):
        ts_rot = R.from_euler('zyx', [z_rot, 180, 0], degrees=True)
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


def main():
    folder_path = 'src/mars_ros/tools/data/ts_2m_box_no_rot_test_05_20230920_143644'
    ts, opti, imu = get_files(folder_path)
    opti = transform_opti(opti)

    #print(alignment_metric(137.862,ts,opti))
    #return
    z_rot = calculate_ts_rotation(ts,opti)
    ts_rot = R.from_euler('zyx', [z_rot, 180, 0], degrees=True)
    aligned_ts = transform_ts(ts, 0, ts_rot.as_matrix(), [0, 0, 0])
    xyz = calculate_ts_translation(aligned_ts,opti)
    aligned_ts = transform_ts(ts, 0, ts_rot.as_matrix(), xyz)
    calculate_ts_offset(aligned_ts, opti,height=5, distance=30)


if __name__ == "__main__":
    main()

