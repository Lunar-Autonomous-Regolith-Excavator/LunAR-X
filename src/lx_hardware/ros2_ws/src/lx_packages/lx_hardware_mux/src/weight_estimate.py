import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lx_msgs.msg import RoverCommand, ToolInfo

class MyNode(Node):
    sensor_msg = None
    def __init__(self):
        super().__init__('weight_estimator')
        self.publisher = self.create_publisher(RoverCommand, '/rover_hw_cmd', 10)
        self.subscription = self.create_subscription(ToolInfo, 'my_topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('Received message: %s' % msg.data)
    
    def publish_drum_pwm(self, data):
        msg = RoverCommand()
        msg.drum_pwm = data
        self.publisher.publish(msg)
    
    def publish_drum_acc(self, data):
        msg = RoverCommand()
        msg.acc_pwm = data
        self.publisher.publish(msg)

mode = 0 # 0 excavate, 1 lifting, 2 rotating for fixed ticks, 3 dumping
# mode switch logic: 
# 0 to 1: number of ticks > excavate_ticks
# 1 to 2: lift for a fixed time
# 2 to 3: number of ticks > rotate_ticks
# 3 to 0: dump for a fixed time
excavate_ticks = 1500000
rotate_ticks = 10000
lift_time = 10
dump_time = 10

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    start_ticks = 
    while rclpy.ok():
        if(mode == 0):
            


        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()