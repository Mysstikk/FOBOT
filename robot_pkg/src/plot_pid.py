import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import sys

# --- CONFIGURACIÓN ---
# Cambia esto por el nombre de la articulación que quieras evaluar
JOINT_NAME = 'joint_hombro' 
TOPIC_NAME = '/joint_trajectory_controller/controller_state'
# ---------------------

class PIDPlotter(Node):
    def __init__(self):
        super().__init__('pid_plotter')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            TOPIC_NAME,
            self.state_callback,
            10)
        
        self.times = []
        self.actual_positions = []
        self.target_positions = []
        self.start_time = None
        self.joint_idx = -1

        self.get_logger().info(f"Esperando datos del controlador para: {JOINT_NAME}...")

    def state_callback(self, msg):
        # Encontrar el índice de la articulación en el array de mensajes
        if self.joint_idx == -1:
            try:
                self.joint_idx = msg.joint_names.index(JOINT_NAME)
                self.get_logger().info(f"¡Articulación encontrada en el índice {self.joint_idx}!")
            except ValueError:
                return

        # Registrar el tiempo cero en el primer mensaje
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        self.times.append(elapsed)
        self.actual_positions.append(msg.actual.positions[self.joint_idx])
        self.target_positions.append(msg.reference.positions[self.joint_idx])

        # Mantener solo los últimos 200 puntos (unos 4 segundos a 50Hz) para que la gráfica fluya
        if len(self.times) > 200:
            self.times.pop(0)
            self.actual_positions.pop(0)
            self.target_positions.pop(0)

def main(args=None):
    rclpy.init(args=args)
    plotter = PIDPlotter()

    # ROS 2 necesita girar (spin) en un hilo separado para no bloquear la gráfica
    thread = threading.Thread(target=rclpy.spin, args=(plotter,))
    thread.start()

    # Configuración de Matplotlib
    fig, ax = plt.subplots(figsize=(8, 5))
    line_actual, = ax.plot([], [], label='Posición Real (Motor)', color='red', linewidth=2)
    line_target, = ax.plot([], [], label='Posición Objetivo (Controlador)', color='blue', linestyle='--')
    
    ax.set_xlabel('Tiempo (segundos)')
    ax.set_ylabel('Posición (radianes)')
    ax.set_title(f'Ajuste PID - {JOINT_NAME}')
    ax.legend()
    ax.grid(True)

    def update(frame):
        if len(plotter.times) > 0:
            line_actual.set_data(plotter.times, plotter.actual_positions)
            line_target.set_data(plotter.times, plotter.target_positions)
            
            # Ajustar la ventana de la gráfica dinámicamente
            ax.set_xlim(plotter.times[0], plotter.times[-1])
            
            # Ajustar el eje Y con un pequeño margen para ver bien el temblor
            min_y = min(min(plotter.actual_positions), min(plotter.target_positions))
            max_y = max(max(plotter.actual_positions), max(plotter.target_positions))
            margin = 0.05 if abs(max_y - min_y) < 0.05 else (max_y - min_y) * 0.2
            ax.set_ylim(min_y - margin, max_y + margin)
            
        return line_actual, line_target

    # Actualizar gráfica cada 50ms
    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()
