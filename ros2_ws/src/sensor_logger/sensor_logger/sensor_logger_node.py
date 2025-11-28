#!/usr/bin/env python3
"""
AgriTech Sensor Logger Node v2.7
================================
- Detección de movimiento mejorada
- Umbral adaptativo
- Posiciones X,Y
- Consumo de tanque
- Color HSV cuando se corta planta
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Bool, Int32
import requests
from datetime import datetime
from enum import Enum
import threading
from queue import Queue
import time


class TrajectoryState(Enum):
    IDLE = 0
    MOVING = 1
    STOPPING = 2


class SensorLoggerNode(Node):
    def __init__(self):
        super().__init__('sensor_logger_node')

        # ==================== PARAMETERS ====================
        self.declare_parameter('api_host', '172.20.10.2')
        self.declare_parameter('api_port', 5074)
        self.declare_parameter('start_threshold', 2.0)      # Para INICIAR trayectoria
        self.declare_parameter('continue_threshold', 0.5)   # Para CONTINUAR moviéndose
        self.declare_parameter('stop_timeout', 3.0)
        self.declare_parameter('min_trajectory_duration', 2.0)
        self.declare_parameter('api_timeout', 5.0)
        self.declare_parameter('calibration_samples', 50)
        self.declare_parameter('position_post_interval', 2.0)

        self.api_host = self.get_parameter('api_host').value
        self.api_port = self.get_parameter('api_port').value
        self.start_threshold = self.get_parameter('start_threshold').value
        self.continue_threshold = self.get_parameter('continue_threshold').value
        self.stop_timeout = self.get_parameter('stop_timeout').value
        self.min_trajectory_duration = self.get_parameter('min_trajectory_duration').value
        self.api_timeout = self.get_parameter('api_timeout').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.position_post_interval = self.get_parameter('position_post_interval').value

        self.base_url = f"http://{self.api_host}:{self.api_port}"

        # ==================== STATE ====================
        self.trajectory_state = TrajectoryState.IDLE
        self.trajectory_confirmed = False
        self.trajectory_start_time = None
        self.last_movement_time = None

        # IMU Integration
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_imu_time = None
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.bias_samples = 0
        self.bias_calibrated = False
        self.last_position_post = 0.0

        # Tank
        self.volume_at_trajectory_start = None
        self.current_volume = 0.0

        # HSV and plant
        self.current_hsv = {'H': 0, 'S': 0, 'V': 0}
        self.nombre_planta = "Planta_Default"

        # Control flags
        self.shutting_down = False
        self.api_available = True
        self.lock = threading.Lock()

        # ==================== API QUEUE ====================
        self.api_queue = Queue()
        self.api_thread = threading.Thread(target=self._api_worker, daemon=True)
        self.api_thread.start()

        # ==================== QOS ====================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, qos)
        self.create_subscription(Float32, '/tank/volume', self.volume_callback, qos)
        self.create_subscription(Int32, '/hsv_h', self.h_callback, 10)
        self.create_subscription(Int32, '/hsv_s', self.s_callback, 10)
        self.create_subscription(Int32, '/hsv_v', self.v_callback, 10)
        self.create_subscription(String, '/plant_name', self.plant_name_callback, 10)
        self.create_subscription(Bool, '/cut_plant', self.cut_callback, 10)

        # ==================== PUBLISHER ====================
        self.status_pub = self.create_publisher(String, '/agritech/status', 10)

        # ==================== TIMERS ====================
        self.create_timer(0.2, self.state_machine_tick)
        self.create_timer(5.0, self._publish_status)
        self.create_timer(30.0, self._check_api_health)

        self._log_startup()

    # ==================== STARTUP ====================

    def _log_startup(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info("   📊 AgriTech Sensor Logger v2.7")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"API: {self.base_url}")
        self.get_logger().info(f"Start threshold: {self.start_threshold} m/s²")
        self.get_logger().info(f"Continue threshold: {self.continue_threshold} m/s²")
        self.get_logger().info("Calibrating IMU (keep robot still)...")

    def _check_api_health(self):
        try:
            requests.get(f"{self.base_url}/Sensores/GetTrajectory", timeout=2.0)
            if not self.api_available:
                self.get_logger().info("✅ API restored")
            self.api_available = True
        except:
            if self.api_available:
                self.get_logger().warn("⚠️ API not reachable")
            self.api_available = False

    # ==================== IMU CALLBACK ====================

    def imu_callback(self, msg: Imu):
        now_sec = time.time()
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # Calibración rápida
        if not self.bias_calibrated:
            alpha = 0.1 if self.bias_samples < 20 else 0.05
            self.accel_bias_x = self.accel_bias_x * (1 - alpha) + ax * alpha
            self.accel_bias_y = self.accel_bias_y * (1 - alpha) + ay * alpha
            self.bias_samples += 1
            
            if self.bias_samples >= self.calibration_samples:
                self.bias_calibrated = True
                self.get_logger().info(
                    f"✅ IMU calibrated: bias=({self.accel_bias_x:.3f}, {self.accel_bias_y:.3f})"
                )
            return

        # Corregir con bias
        ax_corrected = ax - self.accel_bias_x
        ay_corrected = ay - self.accel_bias_y
        
        # Magnitud total de aceleración
        accel_magnitude = (ax_corrected**2 + ay_corrected**2)**0.5

        with self.lock:
            current_state = self.trajectory_state
            
            if current_state == TrajectoryState.IDLE:
                # Umbral ALTO para iniciar (evitar falsos positivos)
                if accel_magnitude > self.start_threshold:
                    self.get_logger().info(f"📍 Movement detected: {accel_magnitude:.2f} m/s²")
                    self._start_trajectory()
                    self.last_movement_time = now_sec
                    
            elif current_state in [TrajectoryState.MOVING, TrajectoryState.STOPPING]:
                # Umbral BAJO para continuar (detectar movimiento suave)
                if accel_magnitude > self.continue_threshold:
                    self.last_movement_time = now_sec
                    if current_state == TrajectoryState.STOPPING:
                        self.trajectory_state = TrajectoryState.MOVING

                # Actualizar posición si confirmado
                if self.trajectory_confirmed:
                    self._update_position(ax_corrected, ay_corrected, now_sec)

    def _update_position(self, ax, ay, now_sec):
        if self.last_imu_time is None:
            self.last_imu_time = now_sec
            return

        dt = now_sec - self.last_imu_time
        self.last_imu_time = now_sec

        if dt > 0.1 or dt < 0.001:
            return

        # Dead zone
        if abs(ax) < 0.15:
            ax = 0.0
        if abs(ay) < 0.15:
            ay = 0.0

        # Integrar
        self.velocity_x += ax * dt
        self.velocity_y += ay * dt

        # Damping
        self.velocity_x *= 0.95
        self.velocity_y *= 0.95

        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt

        # Postear periódicamente
        if now_sec - self.last_position_post >= self.position_post_interval:
            self._post_position()
            self.last_position_post = now_sec

    def _post_position(self):
        if not self.trajectory_confirmed:
            return

        x_cm = round(self.position_x * 100, 2)
        y_cm = round(self.position_y * 100, 2)

        self._queue_api("/Sensores/PostPosition", {"X": x_cm, "Y": y_cm})

    # ==================== SENSOR CALLBACKS ====================

    def volume_callback(self, msg: Float32):
        with self.lock:
            self.current_volume = msg.data
            if self.trajectory_state == TrajectoryState.MOVING and \
               self.volume_at_trajectory_start is None:
                self.volume_at_trajectory_start = msg.data
                self.get_logger().info(f"💧 Tank start: {msg.data:.0f} ml")

    def h_callback(self, msg: Int32):
        with self.lock:
            self.current_hsv['H'] = msg.data

    def s_callback(self, msg: Int32):
        with self.lock:
            self.current_hsv['S'] = msg.data

    def v_callback(self, msg: Int32):
        with self.lock:
            self.current_hsv['V'] = msg.data

    def plant_name_callback(self, msg: String):
        with self.lock:
            self.nombre_planta = msg.data

    def cut_callback(self, msg: Bool):
        if msg.data:
            with self.lock:
                hsv = self.current_hsv.copy()
                nombre = self.nombre_planta

            self._queue_api("/Sensores/color", {
                "NombrePlanta": nombre,
                "H": hsv['H'],
                "S": hsv['S'],
                "V": hsv['V'],
                "FueCortada": True
            })
            self.get_logger().info(f"🔪 Cut: {nombre} HSV({hsv['H']},{hsv['S']},{hsv['V']})")

    # ==================== STATE MACHINE ====================

    def state_machine_tick(self):
        if self.shutting_down:
            return

        now_sec = time.time()

        with self.lock:
            if self.trajectory_state == TrajectoryState.MOVING:
                time_since_movement = now_sec - self.last_movement_time if self.last_movement_time else 0
                if time_since_movement > self.stop_timeout / 2:
                    self.trajectory_state = TrajectoryState.STOPPING
                    self.get_logger().info("⏸️ Slowing down...")

            elif self.trajectory_state == TrajectoryState.STOPPING:
                time_since_movement = now_sec - self.last_movement_time if self.last_movement_time else 0
                if time_since_movement > self.stop_timeout:
                    self._end_trajectory()

    # ==================== TRAJECTORY MANAGEMENT ====================

    def _start_trajectory(self):
        if not self.api_available:
            self.get_logger().warn("API not available")
            return

        self.trajectory_state = TrajectoryState.MOVING
        self.trajectory_start_time = time.time()
        self.volume_at_trajectory_start = self.current_volume
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_imu_time = None
        self.last_position_post = time.time()

        try:
            response = requests.post(
                f"{self.base_url}/Sensores/StartTrajectory",
                json={},
                timeout=self.api_timeout
            )
            if response.status_code in [200, 201]:
                self.trajectory_confirmed = True
                self.get_logger().info("🚀 TRAJECTORY STARTED")
            else:
                self.get_logger().error(f"❌ StartTrajectory failed: {response.status_code}")
                self.trajectory_state = TrajectoryState.IDLE
                self.trajectory_confirmed = False
        except Exception as e:
            self.get_logger().error(f"❌ StartTrajectory error: {e}")
            self.trajectory_state = TrajectoryState.IDLE
            self.trajectory_confirmed = False

    def _end_trajectory(self):
        duration = time.time() - self.trajectory_start_time if self.trajectory_start_time else 0

        if duration < self.min_trajectory_duration:
            self.get_logger().info(f"⏭️ Too short ({duration:.1f}s), discarding")
            self.trajectory_state = TrajectoryState.IDLE
            self.trajectory_confirmed = False
            return

        if self.trajectory_confirmed:
            self._queue_api("/Sensores/CloseTrajectory", {})
            
            x_cm = round(self.position_x * 100, 1)
            y_cm = round(self.position_y * 100, 1)
            self.get_logger().info(f"🏁 TRAJECTORY ENDED ({duration:.1f}s) Final:({x_cm},{y_cm})cm")

            if self.volume_at_trajectory_start is not None and self.current_volume >= 0:
                used = self.volume_at_trajectory_start - self.current_volume
                if used > 0:
                    self._queue_api("/Sensores/Tanque/MLPorDia", {
                        "ML_Al_Inicio": self.volume_at_trajectory_start,
                        "ML_Al_Final": self.current_volume
                    })
                    self.get_logger().info(f"💧 Used: {used:.0f} ml")

        self.trajectory_state = TrajectoryState.IDLE
        self.trajectory_confirmed = False
        self.volume_at_trajectory_start = None
        self.trajectory_start_time = None

    # ==================== API WORKER ====================

    def _api_worker(self):
        while True:
            task = self.api_queue.get()
            if task is None:
                break

            endpoint, payload = task
            url = f"{self.base_url}{endpoint}"

            try:
                response = requests.post(url, json=payload, timeout=self.api_timeout)
                if response.status_code not in [200, 201, 204]:
                    self.get_logger().warn(f"API {response.status_code}: {endpoint}")
            except requests.exceptions.Timeout:
                self.get_logger().warn(f"Timeout: {endpoint}")
            except requests.exceptions.ConnectionError:
                self.api_available = False
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

            self.api_queue.task_done()

    def _queue_api(self, endpoint: str, payload: dict):
        self.api_queue.put((endpoint, payload))

    # ==================== STATUS ====================

    def _publish_status(self):
        if self.shutting_down:
            return

        with self.lock:
            state = self.trajectory_state.name
            confirmed = "✓" if self.trajectory_confirmed else "✗"
            vol = self.current_volume
            imu = "✓" if self.bias_calibrated else f"{self.bias_samples}/{self.calibration_samples}"
            api = "✓" if self.api_available else "✗"
            x = self.position_x * 100
            y = self.position_y * 100

        msg = String()
        msg.data = f"[{state}:{confirmed}] IMU:{imu} API:{api} Tank:{vol:.0f}ml ({x:.1f},{y:.1f})cm"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def destroy_node(self):
        self.shutting_down = True
        if self.trajectory_confirmed:
            try:
                requests.post(f"{self.base_url}/Sensores/CloseTrajectory", json={}, timeout=2.0)
            except:
                pass
        self.api_queue.put(None)
        self.api_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()