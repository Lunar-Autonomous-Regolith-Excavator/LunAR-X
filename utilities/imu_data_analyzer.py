import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.spatial.transform import Rotation as R

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()
        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        data = [ (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]) ) for timestamp,data in rows]
        return np.array(data)
    
def convert_to_rpy(quaternion):
    print(quaternion)
    # convert sensor_msgs/Imu quaternion to roll, pitch, yaw
    rotation_matrix = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    rpy = R.from_matrix(rotation_matrix).as_euler('zyx', degrees=True)
    return rpy
    

if __name__ == "__main__":
    bag_files = []
    # bag_files.append('../bags/ts16_126deg_1_rosbag2_2023_09_27-01_03_31/rosbag2_2023_09_27-01_03_31_0.db3')
    # bag_files.append('../bags/ts16_126deg_2_rosbag2_2023_09_27-01_11_11/rosbag2_2023_09_27-01_11_11_0.db3')
    # bag_files.append('../bags/ts16_45deg_rosbag2_2023_09_27-01_12_35/rosbag2_2023_09_27-01_12_35_0.db3')
    bag_files.append('../bags/localization_fix_orientation_bag/rosbag2_2023_11_02-20_58_09_0.db3')

    imu_datas = []
    ts_datas = []
    for i in range(len(bag_files)):
        print(bag_files[i])
        parser = BagFileParser(bag_files[i])
        imu_datas.append(parser.get_messages("/vectornav/imu")) # of type sensor_msgs/msg/Imu
        ts_datas.append(parser.get_messages("/total_station_pose_map")) # of type geometry_msgs/msg/PoseWithCovarianceStamped


    # plot all acceleration in datas[] 3 different plots
    for i in range(len(bag_files)):
        # plot rpy of IMU and TS
        # use convert to rpy function for both imu and ts
        # convert IMU data into rpy
        imu_rpy = []
        for j in range(len(imu_datas[i])):
            imu_rpy.append(convert_to_rpy(imu_datas[i][j][1].orientation))
        imu_rpy = np.array(imu_rpy)
        imu_rpy = imu_rpy.T
        # convert TS data into rpy
        ts_rpy = []
        for j in range(len(ts_datas[i])):
            ts_rpy.append(convert_to_rpy(ts_datas[i][j][1].pose.pose.orientation))
        ts_rpy = np.array(ts_rpy)
        ts_rpy = ts_rpy.T
        # plot rpy
        plt.figure(i)
        plt.subplot(3,1,1)
        plt.plot(imu_datas[i][:,0], imu_rpy[0], label='imu')
        plt.plot(ts_datas[i][:,0], ts_rpy[0], label='ts')
        plt.legend()
        plt.subplot(3,1,2)
        plt.plot(imu_datas[i][:,0], imu_rpy[1], label='imu')
        plt.plot(ts_datas[i][:,0], ts_rpy[1], label='ts')
        plt.legend()
        plt.subplot(3,1,3)
        plt.plot(imu_datas[i][:,0], imu_rpy[2], label='imu')
        plt.plot(ts_datas[i][:,0], ts_rpy[2], label='ts')
        
    plt.show()