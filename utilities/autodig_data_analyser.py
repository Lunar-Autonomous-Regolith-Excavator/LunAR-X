import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os

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
    # get names of all bag files in ../bags/auto_dig using os.listdir
    all_file_names = os.listdir('../bags/autodig')

    # get all bag files in ../bags/auto_dig
    for file_name in all_file_names:
        bag_files.append(f'../bags/autodig/{file_name}/{file_name}_0.db3')
    
    print(bag_files)
    
    tool_info = []
    tool_height_relative = []
    for i in range(len(bag_files)):
        parser = BagFileParser(bag_files[i])
        tool_info.append(parser.get_messages("/tool_info"))
        tool_height_relative.append(parser.get_messages("/tool_height"))
        
    all_drum_current = []
    all_drum_current_times = []
    all_tool_heights = []
    all_tool_height_times = []
    for i in range(1):
        drum_current = np.array([tool_info[i][j][1].drum_current for j in range(len(tool_info[i]))])
        drum_current_times = np.array([tool_info[i][j][0] for j in range(len(tool_info[i]))])
        tool_heights = np.array([tool_height_relative[i][j][1].data for j in range(len(tool_height_relative[i]))])
        tool_height_times = np.array([tool_height_relative[i][j][0] for j in range(len(tool_height_relative[i]))])
        # offset times by the first time
        drum_current_times -= drum_current_times[0]
        tool_height_times -= tool_height_times[0]
        all_drum_current.append(drum_current)
        all_drum_current_times.append(drum_current_times)
        all_tool_heights.append(tool_heights)
        all_tool_height_times.append(tool_height_times)
    

    # plot current on plot above and height on plot below
    fig = plt.figure(figsize=(15,8))
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    for i in range(len(all_drum_current)):
        ax1.plot(all_drum_current_times[i], all_drum_current[i], label='drum current {}'.format(i), markersize=1)
        ax2.plot(all_tool_height_times[i], all_tool_heights[i], label='tool height {}'.format(i), markersize=1)
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('current (A)')
    ax1.legend()
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('height (m)')
    ax2.legend()
    plt.savefig('current_and_height.png')