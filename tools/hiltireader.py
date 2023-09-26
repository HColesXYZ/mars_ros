import rosbag
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

# Define the input and output bag file paths
input_bag_path = 'src/mars_ros/tools/data/LAB_Survey_2.bag'
output_bag_path = 'src/mars_ros/tools/data/hilti.bag'

# Define the new topics
imu_topic = '/imu_in_topic'
pose_topic = '/pose_in_topic'
# Open the input and output bag files
with rosbag.Bag(input_bag_path, 'r') as in_bag, rosbag.Bag(output_bag_path, 'w') as out_bag:
    for topic, msg, t in in_bag.read_messages():
        if topic == '/alphasense/imu' :#or topic == '/alphasense/imu_adis':
            # Modify the topic of IMU messages
            msg.header.frame_id = imu_topic
            out_bag.write(imu_topic, msg, t)
        elif topic == '/natnet_ros/rigid_bodies/Phasma/pose':
            # Modify the topic of Pose messages
            msg.header.frame_id = pose_topic
            out_bag.write(pose_topic, msg, t)

print("Data extraction and re-saving completed.")
