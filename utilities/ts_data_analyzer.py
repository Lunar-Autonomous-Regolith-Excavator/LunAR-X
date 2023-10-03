import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

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
    

if __name__ == "__main__":
    bag_files = []
    bag_files.append('../bags/ts16_126deg_1_rosbag2_2023_09_27-01_03_31/rosbag2_2023_09_27-01_03_31_0.db3')
    bag_files.append('../bags/ts16_126deg_2_rosbag2_2023_09_27-01_11_11/rosbag2_2023_09_27-01_11_11_0.db3')
    bag_files.append('../bags/ts16_45deg_rosbag2_2023_09_27-01_12_35/rosbag2_2023_09_27-01_12_35_0.db3')
    fig = plt.figure()

    datas = []
    for i in range(len(bag_files)):
        print(bag_files[i])
        parser = BagFileParser(bag_files[i])
        datas.append(parser.get_messages("/total_station_prism"))

    # plot all points in datas[] in 3d
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for data in datas:
        # data is Nxtimestamp xposewithcovariancestamped
        # extract x,y,z position
        x = np.zeros(len(data))
        y = np.zeros(len(data))
        z = np.zeros(len(data))
        for i in range(len(data)):
            x[i] = data[i][1].pose.pose.position.x
            y[i] = data[i][1].pose.pose.position.y
            z[i] = data[i][1].pose.pose.position.z

        # 3d plot  the data
        ax.scatter(x, y, z, marker='o')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        
    plt.show()