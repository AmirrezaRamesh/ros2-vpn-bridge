import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#we receive the data over VPN

#ROS2 Node
class Receive_Data(Node):

    def __init__(self):
        
        super().__init__('subscriber')

        #subscriber
        self.subscriber = self.create_subsribtion(
        String,
        "Message",
        self.listener_callback,
        10)

    #listener callback
    def listener_callback(self, msg):

        self.get_logger().info('Receiving Machine A regards: %s' %msg.data)

#main function
def main(args=None):
    rclpy.init(args=args)

    subscriber = Receive_Data()
    rclpy.spin(subscriber)
    subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

        










