#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# List here any nodes and topics you expect to be up at startup
REQUIRED_NODES = [
    '/stm_node',         # replace with your actual STM32 node name (if any)
    '/sensor_node',      # replace with your actual sensor node name
]

REQUIRED_TOPICS = [
    '/stm32/status',     # replace with the topic name used by STM32
    '/sensor/data',      # replace with your actual sensor topic
]

class InitChecker(Node):
    def __init__(self):
        super().__init__('init_checker')
        self.get_logger().info('[init_checker] Starting system check…')
        self._timer = self.create_timer(2.0, self.check_system)
        self._all_good = False

    def check_system(self):
        # 1) Check for required nodes
        node_list = self.get_node_names()
        missing_nodes = [n for n in REQUIRED_NODES if n not in node_list]

        # 2) Check for required topics
        topic_list = [name for name, _ in self.get_topic_names_and_types()]
        missing_topics = [t for t in REQUIRED_TOPICS if t not in topic_list]

        if missing_nodes or missing_topics:
            if missing_nodes:
                self.get_logger().error(f'Missing nodes: {missing_nodes}')
            if missing_topics:
                self.get_logger().error(f'Missing topics: {missing_topics}')
        else:
            self.get_logger().info('✅ All required nodes and topics found.')
            self._all_good = True
            self._timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitChecker()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
