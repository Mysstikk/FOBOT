#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from transitions import Machine

class Sequencer(Node):
    estados = ['INICIO', 'ESPERA', 'HOMING', 'MODO_AUTO', 'MODO_SERVOING', 'MODO_MANUAL', 'ERROR']
    def __init__(self):
        super().__init__('sequencer_node')
        
        self.state_pub = self.create_publisher(
            String, 
            '/FOBOT/state', 
            10
        )
        
        self.create_subscription(
            String,
            '/FOBOT/movement_done',
            self.movement_done_cb, 
            10
        )
        
        self.create_subscription(
            String,
            '/FOBOT/interface',
            self.interface_cb,
            10
        )

        self.machine = Machine(model=self, states=Sequencer.estados, initial='INICIO')

        # 2. Definimos las reglas (Transiciones)
        # trigger: la orden que das | source: estado origen | dest: estado destino
        self.machine.add_transition(trigger='sistema_listo', source='INICIO', dest='ESPERA')
        
        self.machine.add_transition(trigger='homing', source='ESPERA', dest='HOMING', after='activate_mode')
        
        self.machine.add_transition(trigger='homing_completado', source='HOMING', dest='ESPERA', after='activate_mode')
        
        self.machine.add_transition(trigger='activar_auto', source='ESPERA', dest='MODO_AUTO', after='activate_mode')
        
        self.machine.add_transition(trigger='activar_manual', source='ESPERA', dest='MODO_MANUAL', after='activate_mode')
        
        self.machine.add_transition(trigger='activar_servoing', source='MODO_AUTO', dest='MODO_SERVOING', after='activate_mode')
        
        self.machine.add_transition(trigger='desactivar_manual', source='MODO_MANUAL', dest='ESPERA', after='activate_mode')
        
        self.machine.add_transition(trigger='desactivar_auto', source=['MODO_AUTO', 'MODO_SERVOING'], dest='ESPERA', after='activate_mode')
        
        self.machine.add_transition(trigger='desactivar_servoing', source='MODO_SERVOING', dest='MODO_AUTO', after='activate_mode')
        
        self.machine.add_transition(trigger='emergencia', source='*', dest='ERROR', after='activate_mode') # El '*' es desde cualquier estado
        
        self.machine.add_transition(trigger='rearme', source='ERROR', dest='ESPERA', after='activate_mode')

        self.get_logger().info("Secuenciador iniciado. Estado actual: " + self.state)
        
        self.sistema_listo()
        
        self.state_msg = String()
        
    def interface_cb(self, msg):
        if msg.data == 'homing':
            if self.state == 'ESPERA':
                self.homing()
            else:
                self.get_logger().warning('Homing denegado')
        elif msg.data == 'auto':
            if self.state == 'ESPERA':
                self.activar_auto()
        elif msg.data == 'emergencia':
            self.emergencia()
        elif msg.data == 'rearme':
            if self.state == 'ERROR':
                self.rearme()
            
    def movement_done_cb(self, msg):
        if 'homing' in msg.data:
            self.homing_completado()
        elif 'stop' in msg.data:
            if self.state in ['MODO_AUTO', 'MODO_SERVOING']:
                self.desactivar_auto()
            else:
                self.get_logger().info('Señal de STOP ignorada: El robot ya está en espera o detenido.')
        elif 'manual' in msg.data:
            self.desactivar_manual()
        elif msg.data == 'act_servo':
            self.activar_servoing()
        elif msg.data == 'deact_servo':
            self.desactivar_servoing()
        
    def activate_mode(self):
        self.state_msg.data = self.state
        self.state_pub.publish(self.state_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Sequencer()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
