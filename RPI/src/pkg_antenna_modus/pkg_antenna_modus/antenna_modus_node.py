import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial

class OrientationListener(Node):

    def __init__(self):
        super().__init__('orientation_listener')

        # Serial setup (adjust as needed)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_port = None

        # Subscription to yaw/pitch topic
        self.subscription = self.create_subscription(
            Vector3,
            'antenna_orientation',  # Topic name
            self.orientation_callback,
            10
        )
        self.get_logger().info('Listening on /antenna_orientation for yaw/pitch')

    def orientation_callback(self, msg):
        yaw = msg.x
        pitch = msg.y

        self.get_logger().info(f"Received orientation: yaw={yaw:.2f}, pitch={pitch:.2f}")
        self.send_to_stm(yaw, pitch)

    def send_to_stm(self, yaw, pitch):
        if self.serial_port is not None and self.serial_port.is_open:
            data_str = f"YAW:{yaw:.2f},PITCH:{pitch:.2f}\n"
            self.serial_port.write(data_str.encode('utf-8'))
            self.get_logger().info(f"Sent to STM: {data_str.strip()}")
        else:
            self.get_logger().warn("Serial port is not available.")

def main(args=None):
    rclpy.init(args=args)
    node = OrientationListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
