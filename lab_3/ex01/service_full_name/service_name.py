import rclpy
from rclpy.node import Node
from dmrzlobin_interfaces.srv import FullNameSumService

class ServiceNameNode(Node):
    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(
            FullNameSumService,
            'SummFullName',
            self.handle_service_request
        )
        self.get_logger().info('Service SummFullName started')

    def handle_service_request(self, request, response):
        response.full_name = f"{request.last_name} {request.name} {request.first_name}".strip()
        self.get_logger().info(f'Received: last_name={request.last_name}, name={request.name}, first_name={request.first_name}')
        self.get_logger().info(f'Response: full_name={response.full_name}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceNameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
