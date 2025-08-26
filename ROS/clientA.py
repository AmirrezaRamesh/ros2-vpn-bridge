import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#we send the data over VPN

#ROS2 Node
class Send_data(Node):
    
    def __init__(self):

        super().__init__('publisher')

        #publisher
        self.publisher = self.create_publisher(
            String,
            "Message",
            10)
        
        #timer
        self.timer = self.create_timer(1, self.timer_callback)

    #timer callback
    def timer_callback(self):
        msg = String()
        msg.data = 'VPN worked!'

        self.publisher.publish(msg)

        self.get_logger().info('Sending my regards to Machine B : "%s"' %msg.data)

#main function
def main(args=None):

    rclpy.init(args=args)

    publisher = Send_data()
    rclpy.spin(publisher)
    publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


        



