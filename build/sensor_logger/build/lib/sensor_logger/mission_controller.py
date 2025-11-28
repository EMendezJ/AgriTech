#!/usr/bin/env python3
"""
AgriTech Mission Controller v1.0
================================
Orquesta la máquina de estados principal:

1. IDLE          - Esperando inicio
2. HARVESTING    - Color detector activo (corte de frutas)
3. RETURN_HOME_1 - Regreso después de cosecha
4. WATERING      - Nodo de riego activo
5. RETURN_HOME_2 - Regreso después de riego
6. FINISHED      - Misión completada
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Twist
from enum import Enum
import time


class MissionState(Enum):
    IDLE = 0
    HARVESTING = 1
    RETURN_HOME_1 = 2
    WATERING = 3
    RETURN_HOME_2 = 4
    FINISHED = 5


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # ==================== PARAMETERS ====================
        self.declare_parameter('return_duration', 5.0)  # Segundos para regresar
        self.declare_parameter('return_speed', -0.015)  # Velocidad de retorno (negativa)
        self.declare_parameter('total_plants', 4)

        self.return_duration = self.get_parameter('return_duration').value
        self.return_speed = self.get_parameter('return_speed').value
        self.total_plants = self.get_parameter('total_plants').value

        # ==================== STATE ====================
        self.state = MissionState.IDLE
        self.return_start_time = None
        self.harvesting_complete = False
        self.watering_complete = False

        # ==================== SUBSCRIBERS ====================
        # Iniciar misión
        self.create_subscription(Bool, '/agritech/start_mission', self.start_mission_callback, 10)
        
        # Estado del color detector (recorrido terminado)
        self.create_subscription(String, '/agritech/harvest_status', self.harvest_status_callback, 10)
        
        # Estado del nodo de riego
        self.create_subscription(String, '/agritech/watering_status', self.watering_status_callback, 10)

        # ==================== PUBLISHERS ====================
        # Control de movimiento
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Activar color detector
        self.harvest_start_pub = self.create_publisher(Bool, '/agritech/start_harvest', 10)
        
        # Activar nodo de riego
        self.watering_start_pub = self.create_publisher(Bool, '/agritech/start_watering', 10)
        
        # Estado de la misión
        self.status_pub = self.create_publisher(String, '/agritech/mission_status', 10)

        # ==================== TIMERS ====================
        self.create_timer(0.1, self.state_machine_loop)
        self.create_timer(2.0, self.publish_status)

        self._log_startup()

    def _log_startup(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("   🤖 AgriTech Mission Controller v1.0")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("   State Machine:")
        self.get_logger().info("   ┌─────────────────────────────────────┐")
        self.get_logger().info("   │  IDLE                               │")
        self.get_logger().info("   │    ↓ (start_mission)                │")
        self.get_logger().info("   │  HARVESTING (color_detector)        │")
        self.get_logger().info("   │    ↓ (4 plants checked)             │")
        self.get_logger().info("   │  RETURN_HOME_1                      │")
        self.get_logger().info("   │    ↓                                │")
        self.get_logger().info("   │  WATERING (watering_node)           │")
        self.get_logger().info("   │    ↓ (plants watered)               │")
        self.get_logger().info("   │  RETURN_HOME_2                      │")
        self.get_logger().info("   │    ↓                                │")
        self.get_logger().info("   │  FINISHED                           │")
        self.get_logger().info("   └─────────────────────────────────────┘")
        self.get_logger().info("")
        self.get_logger().info("   Publish to /agritech/start_mission to begin")
        self.get_logger().info("=" * 60)

    # ==================== CALLBACKS ====================

    def start_mission_callback(self, msg: Bool):
        if msg.data and self.state == MissionState.IDLE:
            self.get_logger().info("🚀 MISSION STARTED!")
            self._transition_to(MissionState.HARVESTING)

    def harvest_status_callback(self, msg: String):
        """Detectar cuando el color_detector termina"""
        if "TERMINADO" in msg.data or "FINISHED" in msg.data:
            if self.state == MissionState.HARVESTING:
                self.harvesting_complete = True
                self.get_logger().info("✅ Harvesting complete!")

    def watering_status_callback(self, msg: String):
        """Detectar cuando el watering_node termina"""
        if "COMPLETE" in msg.data or "FINISHED" in msg.data:
            if self.state == MissionState.WATERING:
                self.watering_complete = True
                self.get_logger().info("✅ Watering complete!")

    # ==================== STATE MACHINE ====================

    def state_machine_loop(self):
        if self.state == MissionState.IDLE:
            self._stop_robot()

        elif self.state == MissionState.HARVESTING:
            # El color_detector maneja el movimiento
            if self.harvesting_complete:
                self._transition_to(MissionState.RETURN_HOME_1)

        elif self.state == MissionState.RETURN_HOME_1:
            self._execute_return_home()

        elif self.state == MissionState.WATERING:
            # El watering_node maneja el movimiento
            if self.watering_complete:
                self._transition_to(MissionState.RETURN_HOME_2)

        elif self.state == MissionState.RETURN_HOME_2:
            self._execute_return_home()

        elif self.state == MissionState.FINISHED:
            self._stop_robot()

    def _transition_to(self, new_state: MissionState):
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"📍 State: {old_state.name} → {new_state.name}")

        # Acciones de entrada a cada estado
        if new_state == MissionState.HARVESTING:
            self._start_harvesting()

        elif new_state == MissionState.RETURN_HOME_1:
            self.return_start_time = time.time()
            self.get_logger().info("🔙 Returning home after harvest...")

        elif new_state == MissionState.WATERING:
            self._start_watering()

        elif new_state == MissionState.RETURN_HOME_2:
            self.return_start_time = time.time()
            self.get_logger().info("🔙 Returning home after watering...")

        elif new_state == MissionState.FINISHED:
            self._finish_mission()

    def _start_harvesting(self):
        """Activar el nodo color_detector"""
        self.harvesting_complete = False
        msg = Bool()
        msg.data = True
        self.harvest_start_pub.publish(msg)
        self.get_logger().info("🌱 Starting harvest phase (color_detector)...")

    def _start_watering(self):
        """Activar el nodo watering"""
        self.watering_complete = False
        msg = Bool()
        msg.data = True
        self.watering_start_pub.publish(msg)
        self.get_logger().info("💧 Starting watering phase...")

    def _execute_return_home(self):
        """Ejecutar retorno al inicio (mover hacia atrás)"""
        elapsed = time.time() - self.return_start_time

        if elapsed < self.return_duration:
            # Retroceder
            twist = Twist()
            twist.linear.x = self.return_speed
            self.cmd_pub.publish(twist)
        else:
            # Terminó el retorno
            self._stop_robot()
            
            if self.state == MissionState.RETURN_HOME_1:
                self._transition_to(MissionState.WATERING)
            elif self.state == MissionState.RETURN_HOME_2:
                self._transition_to(MissionState.FINISHED)

    def _stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def _finish_mission(self):
        self._stop_robot()
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("   🏁 MISSION COMPLETE!")
        self.get_logger().info("=" * 60)
        self.get_logger().info("   ✅ Harvesting: Done")
        self.get_logger().info("   ✅ Watering: Done")
        self.get_logger().info("   ✅ Robot: Home")
        self.get_logger().info("=" * 60)

    # ==================== STATUS ====================

    def publish_status(self):
        msg = String()
        msg.data = f"[MISSION] State: {self.state.name}"
        self.status_pub.publish(msg)
        
        if self.state not in [MissionState.IDLE, MissionState.FINISHED]:
            self.get_logger().info(msg.data)

    def destroy_node(self):
        self._stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
