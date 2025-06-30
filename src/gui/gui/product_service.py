import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json
import os
from ament_index_python.packages import get_package_share_directory

class ProductService(Node):
    def __init__(self):
        super().__init__('product_service')
        self.srv = self.create_service(Trigger, 'get_product_info', self.get_product_info)
        self.get_logger().info("🟢 ProductService 서버가 시작되었습니다: 'get_product_info'")

    def get_product_info(self, request, response):
        try:
            # 🔽 install/share/gui/product_data.json 위치에서 불러오기
            json_path = os.path.join(get_package_share_directory('gui'), 'product_data.json')
    
            with open(json_path, 'r') as f:
                data = json.load(f)
    
            result = "\n".join([f"{name}: {price}원" for name, price in data.items()])
            response.success = True
            response.message = result
        except Exception as e:
            response.success = False
            response.message = f"❌ 오류: {str(e)}"
    
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ProductService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
