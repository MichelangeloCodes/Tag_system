import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # or whatever message type you use

class CommandListener(Node):

    def __init__(self):
        super().__init__('command_listener')
        self.subscription = self.create_subscription(
            String,
            'antenna_command',
            self.command_callback,
            10)
        self.get_logger().info('CommandListener started and listening on antenna_command')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if command == 'angle':
            self.send_to_stm(command)
        elif command == 'sweep':
            self.get_logger().info('Sweep mode selected - implement accordingly')
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def send_to_stm(self, command):
        # This function should handle communication with STM.
        # For example, send serial commands or publish to another topic.
        self.get_logger().info(f'Sending command to STM: {command}')
        # TODO: Implement actual communication with STM here

def main(args=None):
    rclpy.init(args=args)
    node = CommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()