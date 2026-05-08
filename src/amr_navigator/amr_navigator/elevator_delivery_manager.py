import math
import os
import time
from typing import Dict, Any, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ClearEntireCostmap


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ElevatorDeliveryManager(Node):
    def __init__(self):
        super().__init__('elevator_delivery_manager')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('start_floor', 1)

        self.declare_parameter('scan_topic', '/rplidar1/scan_filtered')
        self.declare_parameter('current_floor_topic', '/current_floor')
        self.declare_parameter('elevator_start_topic', '/elevator/start')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')

        waypoint_file = str(self.get_parameter('waypoint_file').value)
        self.start_floor = int(self.get_parameter('start_floor').value)

        scan_topic = str(self.get_parameter('scan_topic').value)
        current_floor_topic = str(self.get_parameter('current_floor_topic').value)
        elevator_start_topic = str(self.get_parameter('elevator_start_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        if not waypoint_file:
            waypoint_file = os.path.join(
                get_package_share_directory('amr_navigator'),
                'config',
                'waypoints.yaml'
            )

        with open(waypoint_file, 'r') as f:
            self.wp = yaml.safe_load(f)

        self.scan = None
        self.current_floor = None

        self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.create_subscription(
            Int32,
            current_floor_topic,
            self.floor_callback,
            10
        )

        self.elevator_start_pub = self.create_publisher(
            Int32,
            elevator_start_topic,
            10
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.load_map_client = self.create_client(
            LoadMap,
            '/map_server/load_map'
        )

        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )

        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        self.get_logger().info(
            f"ElevatorDeliveryManager ready. "
            f"waypoint_file={waypoint_file}, "
            f"start_floor={self.start_floor}, "
            f"scan_topic={scan_topic}, "
            f"current_floor_topic={current_floor_topic}, "
            f"elevator_start_topic={elevator_start_topic}, "
            f"cmd_vel_topic={cmd_vel_topic}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def floor_callback(self, msg: Int32):
        self.current_floor = int(msg.data)

    # ------------------------------------------------------------------
    # YAML helpers
    # ------------------------------------------------------------------
    def get_floor_info(self, floor: int) -> Dict[str, Any]:
        maps = self.wp.get('maps', {})

        if floor in maps:
            return maps[floor]

        if str(floor) in maps:
            return maps[str(floor)]

        raise KeyError(f"waypoints.yaml에 maps.{floor} 정보가 없습니다.")

    def get_room_and_target_floor(self, room_number: str) -> Tuple[Dict[str, Any], int]:
        room_number = room_number.strip()

        if not room_number:
            raise ValueError("목적 호수가 비어 있습니다.")

        if not room_number[0].isdigit():
            raise ValueError(
                f"목적 호수 '{room_number}'의 첫 글자가 숫자가 아닙니다. "
                "예: 302, 304, 204"
            )

        target_floor = int(room_number[0])

        if target_floor <= 0:
            raise ValueError(
                f"목적 호수 '{room_number}'에서 추출한 목적층이 {target_floor}입니다. "
                "현재 코드는 1층 이상 목적지만 지원합니다."
            )

        rooms = self.wp.get('rooms', {})

        if room_number not in rooms:
            raise KeyError(
                f"waypoints.yaml의 rooms에 '{room_number}'가 없습니다. "
                f'rooms: "{room_number}": 항목을 추가해야 합니다.'
            )

        room = rooms[room_number]

        yaml_floor = room.get('floor', None)
        if yaml_floor is not None:
            try:
                yaml_floor_int = int(yaml_floor)
                if yaml_floor_int != target_floor:
                    self.get_logger().warning(
                        f"방 번호로 계산한 층={target_floor}, "
                        f"waypoints.yaml에 적힌 floor={yaml_floor_int}. "
                        f"이번 미션에서는 방 번호 앞자리 {target_floor}층을 사용합니다."
                    )
            except Exception:
                self.get_logger().warning(
                    f"waypoints.yaml의 rooms['{room_number}'].floor 값을 정수로 해석할 수 없습니다."
                )

        return room, target_floor

    # ------------------------------------------------------------------
    # Pose / map
    # ------------------------------------------------------------------
    def make_pose(self, pose_dict: Dict[str, Any], frame_id: str = 'map') -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_dict['x'])
        msg.pose.position.y = float(pose_dict['y'])
        msg.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))
        return msg

    def publish_initial_pose(self, pose_dict: Dict[str, Any]):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(pose_dict['x'])
        msg.pose.pose.position.y = float(pose_dict['y'])
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.get_logger().info(
            f"Publishing initial pose: "
            f"x={pose_dict['x']}, y={pose_dict['y']}, yaw={pose_dict['yaw']}"
        )

        for _ in range(20):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def load_map(self, map_yaml_name: str) -> bool:
        map_path = os.path.join(
            get_package_share_directory('amr_navigator'),
            'map',
            map_yaml_name
        )

        self.get_logger().info(f"Loading map: {map_path}")

        while rclpy.ok() and not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /map_server/load_map...")

        req = LoadMap.Request()
        req.map_url = map_path

        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()

        if result is None:
            self.get_logger().error("LoadMap service returned None.")
            return False

        self.get_logger().info(f"LoadMap result: {result.result}")

        self.clear_costmaps()
        self.spin_sleep(1.0)
        return True

    def clear_costmaps(self):
        req = ClearEntireCostmap.Request()

        if self.clear_global_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_global_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Requested global costmap clear.")

        if self.clear_local_costmap_client.wait_for_service(timeout_sec=0.5):
            future = self.clear_local_costmap_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Requested local costmap clear.")

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def go_to_pose(self, pose_dict: Dict[str, Any], name: str = 'goal') -> bool:
        self.get_logger().info(f"Go to {name}: {pose_dict}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(pose_dict)

        self.nav_client.wait_for_server()

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"Goal handle is None: {name}")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected: {name}")
            return False

        self.get_logger().info(f"Goal accepted: {name}")

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(f"Goal result is None: {name}")
            return False

        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal succeeded: {name}")
            return True

        self.get_logger().error(f"Goal failed: {name}, status={status}")
        return False

    # ------------------------------------------------------------------
    # Elevator door by LiDAR
    # ------------------------------------------------------------------
    def is_door_open(
        self,
        center_deg: float = 0.0,
        half_width_deg: float = 12.0,
        open_distance: float = 1.20,
        min_open_ratio: float = 0.60
    ) -> bool:
        if self.scan is None:
            return False

        scan = self.scan
        center = math.radians(center_deg)
        half_width = math.radians(half_width_deg)

        selected = []

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            diff = normalize_angle(angle - center)

            if abs(diff) <= half_width:
                if scan.range_min <= r <= scan.range_max:
                    selected.append(r)

        if len(selected) < 5:
            return False

        open_count = sum(1 for r in selected if r > open_distance)
        ratio = open_count / len(selected)

        return ratio >= min_open_ratio

    def wait_until_door_open(self, label: str = 'door') -> bool:
        self.get_logger().info(f"Waiting until {label} opens...")

        stable_count = 0
        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.is_door_open():
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= 10:
                self.get_logger().info(f"{label} opened.")
                return True

            now = time.time()
            if now - last_log_time > 5.0:
                self.get_logger().info(f"Still waiting for {label}...")
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Elevator floor estimator control
    # ------------------------------------------------------------------
    def start_elevator_floor_estimation(self, target_floor: int):
        self.get_logger().info(
            f"Notify elevator_floor_node: start_floor={self.start_floor}, target_floor={target_floor}"
        )

        msg = Int32()
        msg.data = int(target_floor)

        # floor node가 확실히 받을 수 있도록 여러 번 publish
        for _ in range(15):
            self.elevator_start_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until_target_floor(self, target_floor: int) -> bool:
        self.get_logger().info(f"Waiting target floor: {target_floor}")

        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.current_floor == target_floor:
                self.get_logger().info(f"Arrived at target floor: {target_floor}")
                return True

            now = time.time()
            if now - last_log_time > 3.0:
                self.get_logger().info(
                    f"Current estimated floor={self.current_floor}, target={target_floor}"
                )
                last_log_time = now

        return False

    # ------------------------------------------------------------------
    # Simple rotation
    # ------------------------------------------------------------------
    def rotate_180_simple(self, angular_z: float = 0.35, duration_sec: float = 9.0):
        """
        초기 테스트용 open-loop 180도 회전.
        angular_z=0.35 rad/s 기준 pi rad 회전에 약 9초.

        나중에 odom yaw 기반 closed-loop 회전으로 바꾸는 것이 더 정확합니다.
        """
        self.get_logger().info("Rotate 180 deg")

        twist = Twist()
        twist.angular.z = float(angular_z)

        start = time.time()
        while rclpy.ok() and time.time() - start < duration_sec:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        stop = Twist()
        for _ in range(20):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Rotate 180 deg done.")

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def spin_sleep(self, duration_sec: float):
        end_time = time.time() + duration_sec

        while rclpy.ok() and time.time() < end_time:
            remain = max(0.0, end_time - time.time())
            rclpy.spin_once(self, timeout_sec=min(0.1, remain))

    # ------------------------------------------------------------------
    # Main mission
    # ------------------------------------------------------------------
    def run(self):
        room_number = input("목적 호수를 입력하세요. 예: 302, 304 > ").strip()

        try:
            room, target_floor = self.get_room_and_target_floor(room_number)
            floor1 = self.get_floor_info(self.start_floor)
            target_floor_info = self.get_floor_info(target_floor)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        self.get_logger().info(
            f"Mission requested. room={room_number}, target_floor={target_floor}"
        )

        # 1층 map 로드 및 초기 pose 세팅
        if not self.load_map(floor1['map_yaml']):
            return

        self.publish_initial_pose(floor1['start'])
        self.spin_sleep(2.0)

        # 목적지가 1층인 경우 엘리베이터 없이 바로 이동
        if target_floor == self.start_floor:
            self.get_logger().info(
                f"Target floor is same as start floor: {self.start_floor}. "
                "Skip elevator mission."
            )
            self.go_to_pose(room['pose'], f"room {room_number}")
            return

        # 1층 엘리베이터 앞까지 이동
        if not self.go_to_pose(floor1['elevator_front'], '1F elevator_front'):
            return

        # 1층 엘리베이터 문 열림 대기
        if not self.wait_until_door_open('1F elevator door'):
            return

        # 엘리베이터 안으로 진입
        if not self.go_to_pose(floor1['elevator_inside'], '1F elevator_inside'):
            return

        # 층수 추정 시작.
        # 사용자가 302를 입력했다면 target_floor=3이 publish됩니다.
        self.start_elevator_floor_estimation(target_floor)

        # 문을 바라보도록 180도 회전
        self.rotate_180_simple()

        # 목적 층 도착 대기
        if not self.wait_until_target_floor(target_floor):
            return

        # 목적 층에서 문 열림 대기
        if not self.wait_until_door_open(f'{target_floor}F elevator door'):
            return

        # 목적 층 map으로 전환
        if not self.load_map(target_floor_info['map_yaml']):
            return

        # 목적 층에서 엘리베이터 내부 pose로 AMCL 초기화
        self.publish_initial_pose(target_floor_info['elevator_inside'])
        self.spin_sleep(2.0)

        # 엘리베이터 밖으로 나감
        if not self.go_to_pose(
            target_floor_info['elevator_exit'],
            f'{target_floor}F elevator_exit'
        ):
            return

        # 목적 호수로 이동
        if not self.go_to_pose(room['pose'], f'room {room_number}'):
            return

        self.get_logger().info("Mission complete.")


def main():
    rclpy.init()
    node = ElevatorDeliveryManager()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()