import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap


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
        waypoint_file = self.get_parameter('waypoint_file').value

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
            '/rplidar1/scan_filtered',
            self.scan_callback,
            10
        )

        # 실제 층수 측정 알고리즘 topic 이름으로 수정
        self.create_subscription(
            Int32,
            '/current_floor',
            self.floor_callback,
            10
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav',
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

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def floor_callback(self, msg: Int32):
        self.current_floor = msg.data

    def make_pose(self, pose_dict, frame_id='map') -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_dict['x'])
        msg.pose.position.y = float(pose_dict['y'])
        msg.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))
        return msg

    def publish_initial_pose(self, pose_dict):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(pose_dict['x'])
        msg.pose.pose.position.y = float(pose_dict['y'])
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose_dict['yaw']))

        # AMCL 초기 pose covariance. 처음에는 넉넉하게 둠.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        for _ in range(10):
            self.initialpose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def load_map(self, map_yaml_name: str):
        map_path = os.path.join(
            get_package_share_directory('amr_navigator'),
            'map',
            map_yaml_name
        )

        self.get_logger().info(f'Loading map: {map_path}')

        while not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /map_server/load_map...')

        req = LoadMap.Request()
        req.map_url = map_path

        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f'LoadMap result: {result.result}')

    def go_to_pose(self, pose_dict, name='goal'):
        self.get_logger().info(f'Go to {name}: {pose_dict}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(pose_dict)

        self.nav_client.wait_for_server()

        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {name}')
            return False

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result().result
        self.get_logger().info(f'Goal result for {name}: {result}')
        return True

    def is_door_open(self,
                     center_deg=0.0,
                     half_width_deg=12.0,
                     open_distance=1.20,
                     min_open_ratio=0.60):
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

    def wait_until_door_open(self, label='door'):
        self.get_logger().info(f'Waiting until {label} opens...')

        stable_count = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.is_door_open():
                stable_count += 1
            else:
                stable_count = 0

            # 0.1초 * 10 = 약 1초 동안 연속으로 열림이면 open으로 판단
            if stable_count >= 10:
                self.get_logger().info(f'{label} opened')
                return True

    def rotate_180_simple(self, angular_z=0.35, duration_sec=9.0):
        """
        초기 테스트용 open-loop 180도 회전.
        실제 적용 시 odom yaw 기반 closed-loop 회전으로 바꾸는 것이 좋음.
        angular_z=0.35 rad/s 기준 pi rad 회전에 약 9초.
        """
        self.get_logger().info('Rotate 180 deg')

        twist = Twist()
        twist.angular.z = angular_z

        start = time.time()
        while time.time() - start < duration_sec:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        stop = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_target_floor(self, target_floor: int):
        self.get_logger().info(f'Waiting target floor: {target_floor}')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.current_floor == target_floor:
                self.get_logger().info(f'Arrived at target floor: {target_floor}')
                return True

    def run(self):
        room_number = input('목적 호수를 입력하세요. 예: 304 > ').strip()

        if room_number not in self.wp['rooms']:
            self.get_logger().error(f'Unknown room number: {room_number}')
            return

        room = self.wp['rooms'][room_number]
        target_floor = int(room['floor'])

        floor1 = self.wp['maps'][1]
        target_floor_info = self.wp['maps'][target_floor]

        # 1층 map 로드 및 초기 pose 세팅
        self.load_map(floor1['map_yaml'])
        self.publish_initial_pose(floor1['start'])

        # 1층 엘리베이터 앞까지 이동
        self.go_to_pose(floor1['elevator_front'], '1F elevator_front')

        # 문 열림 대기
        self.wait_until_door_open('1F elevator door')

        # 엘리베이터 안으로 진입
        self.go_to_pose(floor1['elevator_inside'], '1F elevator_inside')

        # 문을 바라보도록 180도 회전
        self.rotate_180_simple()

        # 목적 층 도착 대기
        self.wait_until_target_floor(target_floor)

        # 목적 층에서 문 열림 대기
        self.wait_until_door_open(f'{target_floor}F elevator door')

        # 목적 층 map으로 전환
        self.load_map(target_floor_info['map_yaml'])

        # 목적 층에서 엘리베이터 내부 또는 출구 pose로 초기화
        self.publish_initial_pose(target_floor_info['elevator_inside'])

        # 엘리베이터 밖으로 나감
        self.go_to_pose(target_floor_info['elevator_exit'], f'{target_floor}F elevator_exit')

        # 목적 호수로 이동
        self.go_to_pose(room['pose'], f'room {room_number}')

        self.get_logger().info('Mission complete')


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