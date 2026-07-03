import math
import re
import subprocess
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

# основные переменные для гибкости исполнения скрипта
TARGET_ID = 33
BLOCKED_ID_AFTER_TARGET = 21

GRID_COLS = 6
GRID_ROWS = 6
MARKER_SPACING = 1.0

MAP_ROTATION = 0.0

# медленная скорость = стабильность)
MAX_LINEAR = 0.20
MAX_ANGULAR = 0.35

# поврот до движения(!менять чтобы избежать ошибки поворота)
MOVE_YAW_TOLERANCE = 0.09

# Лидарная безопасность.
FRONT_STOP_DISTANCE = 0.45
FRONT_SLOW_DISTANCE = 0.80

# Команда спавна препятствия на 21 айдишник аруко
# для удобства и синхроннсоти, чтобы не вейтить скрипт до ручного спавна препятсвия
SPAWN_OBSTACLE_COMMAND = [
    "ros2",
    "launch",
    "ar_webots_fms_ros2",
    "spawn_object.launch.py",
    "x:=-3.0",
    "y:=3.0",
    "angle_z:=0.0",
    "object_name:=obstacle",
]


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def marker_position(marker_id):
    # перевод аруко в координаты для стабильной езды робьота
    row = marker_id // GRID_COLS
    col = marker_id % GRID_COLS
    x = -row * MARKER_SPACING
    y = col * MARKER_SPACING
    return x, y


def build_graph(blocked_id=None):
    # граф осседства аруко маркеров
    graph = {}

    for marker_id in range(GRID_ROWS * GRID_COLS):
        if (
            marker_id == blocked_id
        ):  # логика такая, что робот будет уже строить маршрут через заблокированный айдишник
            continue

        row = marker_id // GRID_COLS
        col = marker_id % GRID_COLS
        neighbors = []

        if col < GRID_COLS - 1:
            neighbors.append(marker_id + 1)  # +Y
        if row < GRID_ROWS - 1:
            neighbors.append(marker_id + GRID_COLS)  # -X
        if col > 0:
            neighbors.append(marker_id - 1)  # -Y
        if row > 0:
            neighbors.append(marker_id - GRID_COLS)  # +X

        graph[marker_id] = [n for n in neighbors if n != blocked_id]

    return graph


def shortest_path(graph, start_id, target_id):
    # логика поиска кратчайшего пути
    queue = deque([start_id])
    parent = {start_id: None}

    while queue:
        current = queue.popleft()
        if current == target_id:
            break

        for neighbor in graph.get(current, []):
            if neighbor not in parent:
                parent[neighbor] = current
                queue.append(neighbor)

    if target_id not in parent:
        raise RuntimeError(f"Нет маршрута от {start_id} до {target_id}")

    path = []
    current = target_id
    while current is not None:
        path.append(current)
        current = parent[current]

    return list(reversed(path))


class Mission(Node):
    def __init__(self):
        super().__init__("chvt_2026_rmc2_mission")

        self.cmd_pub = self.create_publisher(Twist, "/RMC2/cmd_vel", 10)

        self.create_subscription(String, "/RMC2/aruco_id", self.on_aruco, 10)
        self.create_subscription(Odometry, "/RMC2/odometry", self.on_odom, 10)
        self.create_subscription(LaserScan, "/RMC2/scan", self.on_scan, 10)
        self.create_subscription(
            Bool, "/chvt/emergency_stop", self.on_emergency_stop, 10
        )

        self.current_aruco = None
        self.pose = None
        self.yaw = None

        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")
        self.emergency_stop = False

        self.start_id = None
        self.route_to_target = None
        self.route_to_start = None
        self.route = None
        self.route_index = 0

        self.phase = "WAIT_START_MARKER"

        # Якорь для перевода координат поля в odometry.
        self.anchor_odom_x = None
        self.anchor_odom_y = None
        self.anchor_odom_yaw = None
        self.anchor_marker_x = None
        self.anchor_marker_y = None

        self.last_linear = 0.0
        self.last_angular = 0.0
        self.obstacle_spawned = False
        self.spawn_process = None

        log_dir = Path.home() / "scripts" / "chvt-2026"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = (
            log_dir / f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        )

        self.timer = self.create_timer(0.1, self.tick)

        self.log("MISSION STARTED")
        self.log("Жду стартовую ArUco-метку. До обнаружения метки робот НЕ едет.")

    def on_aruco(self, msg):
        # В сообщении может быть просто "0" или строка с текстом поэтомуберем первое число.
        match = re.search(r"-?\d+", msg.data)
        if match:
            self.current_aruco = int(match.group(0))

    def on_odom(self, msg):
        self.pose = msg.pose.pose.position
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_scan(self, msg):
        # Используем лидар как простую защиту от столкновений.
        count = len(msg.ranges)
        if count == 0:
            return

        center = count // 2
        sector = max(4, count // 14)

        def min_range(start, end):
            values = []
            for i in range(max(0, start), min(count, end)):
                r = msg.ranges[i]
                if math.isfinite(r) and r > 0.02:
                    values.append(r)
            return min(values) if values else float("inf")

        self.front_min = min_range(center - sector, center + sector)
        self.left_min = min_range(center + sector, center + 4 * sector)
        self.right_min = min_range(center - 4 * sector, center - sector)

    def on_emergency_stop(self, msg):
        self.emergency_stop = msg.data
        self.log(f"Ручной аварийный стоп: {self.emergency_stop}")

    def tick(self):
        if self.emergency_stop:
            self.stop()
            return

        if self.pose is None or self.yaw is None:
            self.stop()
            return

        if self.phase == "WAIT_START_MARKER":
            self.wait_start_marker()
        elif self.phase in ("GO_TARGET", "GO_START"):
            self.follow_route()
        elif self.phase == "SPAWN_OBSTACLE":
            self.spawn_obstacle_and_prepare_return()
        elif self.phase == "FINISHED":
            self.stop()

    def wait_start_marker(self):
        self.stop()

        if self.current_aruco is None:
            return

        self.start_id = self.current_aruco
        graph = build_graph()

        if self.start_id not in graph:
            self.log(f"Ошибка: стартовая метка {self.start_id} вне графа")
            return

        self.anchor_odom_x = self.pose.x
        self.anchor_odom_y = self.pose.y
        self.anchor_odom_yaw = self.yaw
        self.anchor_marker_x, self.anchor_marker_y = marker_position(self.start_id)

        self.route_to_target = shortest_path(graph, self.start_id, TARGET_ID)
        self.route = self.route_to_target
        self.route_index = 1 if len(self.route) > 1 else 0
        self.phase = "GO_TARGET"

        self.log(f"Стартовая метка: {self.start_id}")
        self.log(f"Целевая метка: {TARGET_ID}")
        self.log(f"Координаты цели: {marker_position(TARGET_ID)}")
        self.log(f"Маршрут к цели: {self.route_to_target}")
        print(f"ROUTE_TO_TARGET: {self.route_to_target}")

    def follow_route(self):
        if self.route_index >= len(self.route):
            self.stop()

            if self.phase == "GO_TARGET":
                self.log(f"TARGET_REACHED marker={TARGET_ID}")
                print(f"TARGET_REACHED marker={TARGET_ID}")
                self.phase = "SPAWN_OBSTACLE"
            else:
                self.log(f"START_REACHED marker={self.start_id}")
                print(f"START_REACHED marker={self.start_id}")
                self.finish()
            return

        target_marker = self.route[self.route_index]

        # если нижняя камера увидела нужный аруко, значит робот достигнул целевой.
        if self.current_aruco == target_marker:
            self.log(f"Waypoint достигнут по ArUco: {target_marker}")
            self.route_index += 1
            self.stop()
            return

        target_x, target_y = self.marker_to_odom(target_marker)
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - self.yaw)

        # Сначала поворачиваем почти ровно на следующую метку, потом едем вперед.
        if abs(yaw_error) > MOVE_YAW_TOLERANCE:
            linear = 0.0
        elif distance < 0.18:
            # Если почти доехали, но камера еще не увидела ID, медленно доползаем.
            linear = 0.04
        else:
            linear = clamp(0.25 * distance, 0.05, MAX_LINEAR)
            if abs(yaw_error) > 0.05:
                linear *= 0.5

        angular = clamp(0.85 * yaw_error, -MAX_ANGULAR, MAX_ANGULAR)
        if abs(yaw_error) < 0.03:
            angular = 0.0

        # После спавна препятствия маршрут уже построен вокруг 21 айдишник (где как раз и припятсвие)
        # Лидар остается как дополнительная страховка от столкновения.
        if self.front_min < FRONT_STOP_DISTANCE:
            turn = 0.35 if self.left_min > self.right_min else -0.35
            self.publish_cmd(0.0, turn)
            self.log_throttled(f"Препятствие спереди {self.front_min:.2f} м, поворот")
            return

        if self.front_min < FRONT_SLOW_DISTANCE:
            linear *= 0.4

        self.publish_cmd(linear, angular)

    def spawn_obstacle_and_prepare_return(self):
        self.stop()

        if not self.obstacle_spawned:
            self.log(
                f"Спавню препятствие на ID {BLOCKED_ID_AFTER_TARGET}: x=-3.0 y=3.0"
            )
            print("SPAWN_OBSTACLE_ON_ID_21")
            self.spawn_process = subprocess.Popen(SPAWN_OBSTACLE_COMMAND)
            self.obstacle_spawned = True

            # После появления препятствия строим новый обратный маршрут, исключая 21 айдишник
            graph_without_obstacle = build_graph(blocked_id=BLOCKED_ID_AFTER_TARGET)
            self.route_to_start = shortest_path(
                graph_without_obstacle, TARGET_ID, self.start_id
            )
            self.route = self.route_to_start
            self.route_index = 1 if len(self.route) > 1 else 0

            self.log(
                f"Маршрут домой без ID {BLOCKED_ID_AFTER_TARGET}: {self.route_to_start}"
            )
            print(f"ROUTE_TO_START: {self.route_to_start}")

        self.phase = "GO_START"

    def marker_to_odom(self, marker_id):
        # Перевод координат маркера из карты поля в odometry.
        # Стартовая метка является якорем, а стартовый yaw задает поворот карты.
        marker_x, marker_y = marker_position(marker_id)
        dx = marker_x - self.anchor_marker_x
        dy = marker_y - self.anchor_marker_y

        # Робот на старте смотрит в сторону -X поля, поэтому добавляем pi.
        theta = self.anchor_odom_yaw + math.pi + MAP_ROTATION

        odom_dx = dx * math.cos(theta) - dy * math.sin(theta)
        odom_dy = dx * math.sin(theta) + dy * math.cos(theta)
        return self.anchor_odom_x + odom_dx, self.anchor_odom_y + odom_dy

    def finish(self):
        self.phase = "FINISHED"
        self.stop()
        self.log("MISSION FINISHED")
        print("MISSION FINISHED")
        print(f"LOG_FILE: {self.log_path}")

    def publish_cmd(self, linear, angular):
        # Ограничение ускорения убирает резкие движения для стабильной работы робота
        linear = self.last_linear + clamp(linear - self.last_linear, -0.03, 0.03)
        angular = self.last_angular + clamp(angular - self.last_angular, -0.05, 0.05)

        self.last_linear = linear
        self.last_angular = angular

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def log(self, text):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{stamp}] {text}"
        self.get_logger().info(text)

        with self.log_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")

    def log_throttled(self, text):
        now = self.get_clock().now().nanoseconds / 1e9
        last = getattr(self, "_last_log_time", 0.0)
        if now - last > 1.0:
            self._last_log_time = now
            self.log(text)


def main():
    rclpy.init()
    node = Mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log("Остановка через ctrl+c")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
