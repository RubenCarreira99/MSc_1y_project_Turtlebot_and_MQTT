import rclpy, threading, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
from rclpy.executors import SingleThreadedExecutor

msg_json = ""

class BatteryStateSubscriber(Node):
    """
    Subscriber node to the current battery state
    """     
    def __init__(self):
   
      # Initialize the class using the constructor
      super().__init__('battery_state_subscriber')
      self.get_logger().info("Node Launched")
      self.mqttclient = mqtt.Client("ros2mqtt21")
      self.mqttclient.connect("127.0.0.1", 8884, 60) 
     
      self.subscription_battery_state = self.create_subscription(
        BatteryState,
        '/r3/battery_state',
        self.get_battery_state,
        10)
       
    def get_battery_state(self, msg: BatteryState):
        time.sleep(1)
        self.get_logger().info(
            'Battery percentage:{:.0%}'.format(msg.percentage))
        state_pub = int(msg.percentage)
        topic_name = "/r3/bat_state"
        result = self.mqttclient.publish(topic_name, state_pub)
        status = result[0]
        if status == 0:
            print(f"Send `{state_pub}` to topic `{topic_name}`")
        else:
            print(f"Failed to send message to topic `{topic_name}`")
        


class CarTeleop(Node):
    def __init__(self):
        super().__init__('car_teleop')
        self.get_logger().info("Node Launched")
        self.pub = self.create_publisher(Twist, '/r3/cmd_vel', 10)

        self.t = threading.Thread(target=self.set_mqqt)
        self.t.daemon = True
        self.t.start()   

    def on_message(self, client, userdata, msg):
        try:
            msg_json = json.loads(msg.payload)
            self.get_logger().info("vel. linear = " + str(msg_json["linear"]) + " vel. angular = "+str(msg_json["angular"]))
        except Exception as e:
            print("Couldn't parse raw data: %s" % msg.payload, e)

        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        if msg_json != "":

            self.get_logger().info('Received: "%s"' % msg_json)

            control_linear_vel = msg_json["linear"]
            control_angular_vel = msg_json["angular"]

            twist = Twist()

            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            self.pub.publish(twist)
            self.get_logger().info('Publishing: "%s"' % twist)

    def set_mqqt(self):
        self.mqttclient = mqtt.Client("ros2mqtt")
        self.mqttclient.on_message = self.on_message 
        self.mqttclient.connect("127.0.0.1", 8884, 60)  
        self.mqttclient.subscribe("/r3/control", qos=2) 
        self.mqttclient.loop_forever()

def main():
    rclpy.init(args=None)

    try:
        teleop_node = CarTeleop()
        battery_node = BatteryStateSubscriber()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop_node)
        executor.add_node(battery_node)
    
        try:
            executor.spin()
        finally:
            executor.shutdown()
            teleop_node.destroy_node()
            battery_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
