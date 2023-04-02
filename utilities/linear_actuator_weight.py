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
        data = [ (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]).data[3] ) for timestamp,data in rows]
        return np.array(data)
    

if __name__ == "__main__":
    bag_files = [
                (0, '../bags/freeWt_rosbag2_2023_03_29-20_44_14/rosbag2_2023_03_29-20_44_14_0.db3'),
                # (5, '../bags/5_lbs_rosbag2_2023_03_29-20_46_31/rosbag2_2023_03_29-20_46_31_0.db3'),
                # (7.5, '../bags/7.5_lbs_rosbag2_2023_03_29-20_50_33/rosbag2_2023_03_29-20_50_33_0.db3'),
                (10, '../bags/10_lbs_rosbag2_2023_03_29-20_52_27/rosbag2_2023_03_29-20_52_27_0.db3'),
                (12.5, '../bags/12.5_lbs_rosbag2_2023_03_29-20_53_41/rosbag2_2023_03_29-20_53_41_0.db3')
                ]
    parser = []
    for i in range(len(bag_files)):
        parser.append(BagFileParser(bag_files[i][1]))
    # plot in 5x1 subplots
    # fig, axs = plt.subplots(1, 1, sharex=True, sharey=True)
    for i in range(len(bag_files)):
        tester = parser[i].get_messages("/tool_raw_info")
        data = tester[:,1]
        # filtered = signal.savgol_filter(data, 51, 3)
        # filtered = data
        # apply moving average filter on the data
        # filtered = np.convolve(data, np.ones((51,))/51, mode='valid')
        
        # complementary filter
        filtered = np.zeros(len(data))
        filtered[0] = 512
        for j in range(1,len(data)):
            filtered[j] = 0.1*data[j] + 0.9*filtered[j-1]
        filtered = (filtered-512)*(1.5/60)
        plt.plot(np.linspace(0, (len(filtered)+1)/10, len(filtered)), filtered, label=str(bag_files[i][0])+" lbs")
    plt.title("Linear Actuator Current vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")
    plt.legend()
    plt.show()