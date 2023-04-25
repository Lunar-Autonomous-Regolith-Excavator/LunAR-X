#!/usr/bin/env python

import rospy
import subprocess

class ArduinoWatchdogNode:

    def __init__(self):
        rospy.init_node('arduino_watchdog', anonymous=True)
        self.topic_name = 'tool_raw_info'
        self.timeout_duration = 1 # restart if not received message in 3 seconds
        self.last_msg_time = rospy.Time.now()
        rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.topic_callback)

    def topic_callback(self, msg):
        self.last_msg_time = rospy.Time.now()

    def check_timeout(self):
        if rospy.Time.now() - self.last_msg_time > rospy.Duration.from_sec(self.timeout_duration):
            rospy.logwarn('Timeout on topic {}'.format(self.topic_name))
            subprocess.call(['rosnode', 'kill', '-a'])
            rospy.loginfo('Restarting nodes...')
            subprocess.Popen(['roslaunch', 'lx_arduino_handler', 'arduino_mega_handler.launch'])

    def run(self):
        while not rospy.is_shutdown():
            self.check_timeout()
            rospy.sleep(0.5)

if __name__ == '__main__':
    node = ArduinoWatchdogNode()
    # launch arduino nano and mega initially
    subprocess.Popen(['roslaunch', 'lx_arduino_handler', 'arduino_nano_handler.launch'])
    subprocess.Popen(['roslaunch', 'lx_arduino_handler', 'arduino_mega_handler.launch'])

    node.run()
