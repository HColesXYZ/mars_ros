import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

# Define the bag file path
bag_path = 'src/mars_ros/tools/data/hilti.bag'

# Initialize lists to store data
pose_x, pose_y, pose_z = [], [], []
imu_ax, imu_ay, imu_az = [], [], []
imu_gx, imu_gy, imu_gz = [], [], []
pose_qx, pose_qy, pose_qz, pose_qw = [], [], [], []

# Read the bag file
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == '/imu_in_topic':
            imu_data = msg
            imu_ax.append(imu_data.linear_acceleration.x)
            imu_ay.append(imu_data.linear_acceleration.y)
            imu_az.append(imu_data.linear_acceleration.z)
            imu_gx.append(imu_data.angular_velocity.x)
            imu_gy.append(imu_data.angular_velocity.y)
            imu_gz.append(imu_data.angular_velocity.z)
        elif topic == '/pose_in_topic':
            pose_data = msg.pose.position
            pose_x.append(pose_data.x)
            pose_y.append(pose_data.y)
            pose_z.append(pose_data.z)
            pose_q = msg.pose.orientation
            pose_qx.append(pose_q.x)
            pose_qy.append(pose_q.y)
            pose_qz.append(pose_q.z)
            pose_qw.append(pose_q.w)

# Create plots
plt.figure(figsize=(10, 5))

# Plot pose x, y, z
plt.subplot(2, 2, 1)
plt.plot(pose_x, label='Pose X')
plt.plot(pose_y, label='Pose Y')
plt.plot(pose_z, label='Pose Z')
plt.legend()

# Plot imu linear accelerations
plt.subplot(2, 2, 2)
plt.plot(imu_ax, label='IMU Ax')
plt.plot(imu_ay, label='IMU Ay')
plt.plot(imu_az, label='IMU Az')
plt.legend()

# Plot imu angular velocities
plt.subplot(2, 2, 3)
plt.plot(imu_gx, label='IMU Gx')
plt.plot(imu_gy, label='IMU Gy')
plt.plot(imu_gz, label='IMU Gz')
plt.legend()

# Plot pose quaternions
plt.subplot(2, 2, 4)
plt.plot(pose_qx, label='Pose Qx')
plt.plot(pose_qy, label='Pose Qy')
plt.plot(pose_qz, label='Pose Qz')
plt.plot(pose_qw, label='Pose Qw')
plt.legend()

# Show the plots
plt.tight_layout()
plt.show()
