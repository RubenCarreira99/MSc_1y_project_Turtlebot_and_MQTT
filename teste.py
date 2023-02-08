import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String


class NodeA(Node):

    def __init__(self):
        super().__init__('NodeA')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class NodeB(Node):

    def __init__(self):
        super().__init__('NodeB')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=None)

    try:        
        node_a = NodeA()
        node_b = NodeB()

        executor = SingleThreadedExecutor()
        executor.add_node(node_a)
        executor.add_node(node_b)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node_a.destroy_node()
            node_b.destroy_node()
    
    finally:
        rclpy.shutdown()    
    

if __name__ == '__main__':
    main()