import rclpy
from rclpy.node import Node
from dmrzlobin_interfaces.srv import FullNameSumService
import sys

class ClientNameNode(Node):
    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(FullNameSumService, 'SummFullName')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service SummFullName not available, waiting...')
        self.get_logger().info('Client connected to SummFullName service')

    def send_request(self, last_name, name, first_name):
        req = FullNameSumService.Request()
        req.last_name = last_name
        req.name = name
        req.first_name = first_name
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    if len(sys.argv) != 4:
        print('Usage: ros2 run service_full_name client_name <last_name> <name> <first_name>')
        return

    rclpy.init(args=args)
    node = ClientNameNode()
    try:
        response = node.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
        node.get_logger().info(
            f'Request: last_name={sys.argv[1]}, name={sys.argv[2]}, first_name={sys.argv[3]}'
        )
        node.get_logger().info(f'Response: full_name={response.full_name}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
