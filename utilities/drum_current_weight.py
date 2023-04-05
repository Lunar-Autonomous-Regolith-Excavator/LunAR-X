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
        data = [ (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]).data[2], deserialize_message(data, self.topic_msg_message[topic_name]).data[0] ) for timestamp,data in rows]
        return np.array(data)
    

if __name__ == "__main__":
    bag_files = [
                (0, '../bags/rev_0_rosbag2_2023_03_30-23_27_03/rosbag2_2023_03_30-23_27_03_0.db3', 60),
                (3, '../bags/rev_3_rosbag2_2023_03_30-23_28_55/rosbag2_2023_03_30-23_28_55_0.db3', 176),
                # (6, '../bags/rev_6_rosbag2_2023_03_30-23_29_53/rosbag2_2023_03_30-23_29_53_0.db3',300),
                (9, '../bags/rev_9_rosbag2_2023_03_30-23_32_09/rosbag2_2023_03_30-23_32_09_0.db3', 405),
                ]
    parser = []
    for i in range(len(bag_files)):
        parser.append(BagFileParser(bag_files[i][1]))
    # plot in 5x1 subplots
    integrals = []
    fig, axs = plt.subplots(2, 1, constrained_layout=True)
    for i in range(len(bag_files)):
        tester = parser[i].get_messages("/tool_raw_info")
        current_raw = tester[:,1]
        # filtered = signal.savgol_filter(data, 51, 3)
        # filtered = data
        # apply moving average filter on the data
        # filtered = np.convolve(data, np.ones((51,))/51, mode='valid')
        
        # complementary filter
        filtered_current = np.zeros(len(current_raw))
        filtered_current[0] = current_raw[0]
        for j in range(1,len(current_raw)):
            filtered_current[j] = 0.1*current_raw[j] + 0.9*filtered_current[j-1]
        filtered_current = (1.5/60)*(filtered_current - 512)
        
        # get the omega
        omegas = np.zeros(len(current_raw))
        for j in range(1,len(current_raw)):
            omegas[j] = (2.2e6/1.25)*(tester[j,2] - tester[j-1, 2])/(tester[j,0] - tester[j-1, 0])

        i_omega =filtered_current*omegas
        filtered_i_omega = np.zeros(len(i_omega))
        filtered_i_omega[0] = i_omega[0]
        for j in range(1,len(i_omega)):
            filtered_i_omega[j] = 0.2*i_omega[j] + 0.8*filtered_i_omega[j-1]
    
        start_ind = bag_files[i][2]
        add_ind = 60
        axs[0].plot(np.linspace(0, add_ind/10, add_ind), filtered_current[start_ind:start_ind+add_ind], label=str(bag_files[i][0])+" Excavation Rotations")
        axs[1].plot(np.linspace(0, add_ind/10, add_ind), omegas[start_ind:start_ind+add_ind], label=str(bag_files[i][0])+" Excavation Rotations")
        # axs[2].plot(np.linspace(0, add_ind/10, add_ind), i_omega[start_ind:start_ind+add_ind], label=str(bag_files[i][0])+" rotations")
        integrals.append((np.sum(filtered_current[start_ind:start_ind+add_ind]), np.sum(omegas[start_ind:start_ind+add_ind]), 
                            np.sum(i_omega[start_ind:start_ind+add_ind])))
        # axs[2].plot(filtered_i_omega, label=str(bag_files[i][0])+" rotations")

    integrals = np.array(integrals)
    print(integrals.shape)
    axs[0].set_title("Current"); axs[0].set_xlabel("Time (s)"); axs[0].set_ylabel("Current (A)")
    axs[1].set_title("Angular Velocity"); axs[1].set_xlabel("Time (s)"); axs[1].set_ylabel("Omega (rad/s)")
    # axs[2].set_title("Current x Angular Velocity")
    # axs[2].set_ylim([-1e2, 1e7])
    axs[0].legend(); axs[1].legend; 
    # axs[2].legend()

    # bar plot of the integrals
    plt.figure()
    plt.bar(np.arange(integrals.shape[0]), np.abs(integrals[:,0]), label="Current")
    # set xticks of plot
    plt.xticks(np.arange(integrals.shape[0]), [str(bag_files[i][0]) for i in range(len(bag_files))])
    plt.ylabel("Integral of Current (A)")
    plt.xlabel("Number of Excavation Rotations")
    # axs[1].bar(np.arange(integrals.shape[0]), np.abs(integrals[:,1]))
    # axs[2].bar(np.arange(integrals.shape[0]), np.abs(integrals[:,2]))
    plt.show()
