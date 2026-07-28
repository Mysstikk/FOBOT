import rclpy
from rclpy.node import Node
# Importamos el servicio específico del paquete de dynamixel
from dynamixel_interfaces.srv import SetDataToDxl

class TorqueEnabler(Node):
    def __init__(self):
        super().__init__('torque_enabler')
        # Nos conectamos al servicio maestro de escritura
        self.client = self.create_client(SetDataToDxl, '/dynamixel_hardware_interface/set_dxl_data')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio Dynamixel...')
        
        # Encendemos los motores del 1 al 6
        for motor_id in range(1,2):
            req = SetDataToDxl.Request()
            req.id = motor_id
            req.item_name = 'Torque Enable'
            req.item_data = 1
            
            # Enviamos la petición
            self.future = self.client.call_async(req)
            self.get_logger().info(f'Enviada señal de encendido al motor {motor_id}')

def main(args=None):
    rclpy.init(args=args)
    node = TorqueEnabler()
    
    # Damos tiempo a que se envíen los 6 mensajes
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.get_logger().info('¡Todos los motores deberían estar rígidos!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
