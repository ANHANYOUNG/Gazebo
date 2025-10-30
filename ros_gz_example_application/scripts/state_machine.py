#!/usr/bin/env python3

from std_msgs.msg import String, Bool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
import yaml
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy



class StateManager(Node):
    def __init__(self):
    # ========================================================================= #
    # 1. 초기 설정 (__init__)
    # ========================================================================= #
        # 1-1. 노드 초기화
        super().__init__('state_manager_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True
                        ) # 노드 이름 설정
        self.get_logger().info('State Manager has been started.')
        self.declare_parameter('initial_state', 'IDLE') # 초기 상태 파라미터(initial_state=IDLE) 선언
        self.state = self.get_parameter('initial_state').get_parameter_value().string_value # 파라미터 값 읽고 self.state에 저장
        self.timer = self.create_timer(0.1, self.state_machine_loop) #0.1s 마다 state_machine_loop 호출
        self.get_logger().info(f'Initial state: {self.state}') # 초기 상태 로그 출력
        # 1-2. path.yaml 파일용 parameter 설정
        self.available_paths = {}
        # 'paths' 접두사로 시작하는 모든 파라미터(dict: {'path_A': Parameter, ...})
        path_params = self.get_parameters_by_prefix('paths')
        for subkey, param in path_params.items():
            # subkey가 곧 'path_A', 'path_B'
            path_key = subkey
            # ROS2 파라미터 배열은 list/tuple일 수 있으니 list로 캐스팅
            flat_list = list(param.value) if param.value is not None else []
            if not flat_list:
                self.get_logger().error(f"Path '{path_key}' is empty. Skipping.")
                continue
            if len(flat_list) % 2 != 0:
                self.get_logger().error(f"Path '{path_key}' has odd number of elements. Skipping.")
                continue
            # 2개씩 (x,y)로 묶기 + float 변환
            grouped = [[float(v) for v in flat_list[i:i+2]] for i in range(0, len(flat_list), 2)]
            self.available_paths[path_key] = grouped
        if self.available_paths:
            self.get_logger().info(f"Available paths loaded: {list(self.available_paths.keys())}")
        else:
            self.get_logger().warn("'paths.*' parameters not found. Please check path.yaml.")
        


            
    # ========================================================================= #
    # 2. Subscription & Publication 설정
    # ========================================================================= #
        # 1. 상태 전환 명령을 받기 위한 '/state_command' 구독자 생성
        self.command_subscriber = self.create_subscription(String,'/state_command',self.command_callback,10)
        # 2. 수동 개입 감지를 위한 '/cmd_vel' 구독자 생성
        self.manual_override_subscriber = self.create_subscription(Twist,'/cmd_vel',self.manual_override_callback,10)
        self.manual_override_detected = False
        # 3. 속도 명령을 받기 위한 '/cmd_vel' 퍼블리셔 생성
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.zero_twist = Twist()  # 모든 필드 0
        # 4. /ppc/path 퍼블리셔
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 새 구독자에게 마지막 메시지 즉시 전달
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.ppc_path_pub = self.create_publisher(Path, '/ppc/path', path_qos)

        # --- AUTO 모드 하위 상태 및 선택 경로 보관 변수 ---
        self.auto_phase = 'PENDING'          # 'PENDING' or 'RUNNING'
        self.selected_path_name = None       # 예: 'path_A'
        self.selected_path_points = None     # 예: [[x,y], [x,y], ...]
        # 5. /ppc/enable 퍼블리셔
        self.ppc_enable_pub = self.create_publisher(Bool, '/ppc/enable', 10)
        self.ppc_enable_pub.publish(Bool(data=False))  # 시작 시 비활성


    # ========================================================================= #
    # 3. 콜백 함수 정의
    # ========================================================================= #
    # 3-1. 상태 전환 명령 처리 콜백 함수
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # 규칙 1. 어떤 상태에 있든 'E_STOP' 명령을 받으면 즉시 IDLE 상태로.
        if command == 'E_STOP':
            self.get_logger().warn('Emergency Stop')
            self.state = 'IDLE'
            self.stop_pub.publish(self.zero_twist)  # 즉시 정지 명령
            self.ppc_enable_pub.publish(Bool(data=False))  # PPC 비활성화
            return

        # IDLE 상태일 때
        if self.state == 'IDLE':
            self.get_logger().info('State: IDLE', throttle_duration_sec=1.0)
            if command == 'MANUAL':
                self.state = 'MANUAL'
            elif command == 'AUTO':
                self.state = 'AUTO'
                self.auto_phase = 'PENDING'  # AUTO 진입 시 PENDING으로 초기화
                self.selected_path_name = None
                self.selected_path_points = None
                self.get_logger().info('Entered AUTO mode. Waiting for path selection.')
            elif command == 'CAL':
                self.state = 'CALIBRATION'

        # MANUAL 상태일 때
        elif self.state == 'MANUAL':
            self.get_logger().info('State: MANUAL', throttle_duration_sec=1.0)
            if command == 'DONE':
                self.state = 'IDLE'

        # AUTO 상태일 때
        elif self.state == 'AUTO':
            self.get_logger().info('State: AUTO', throttle_duration_sec=1.0)
            # 경로 선택 (정확히 일치하는 문자열만 허용)
            if command == 'path_A':
                if 'path_A' in self.available_paths:
                    self.selected_path_name = 'path_A'
                    self.selected_path_points = self.available_paths['path_A']
                    self.auto_phase = 'PENDING'
                    self.get_logger().info(f'Selected path_A ({len(self.selected_path_points)} pts). Ready to AUTO_START.')
                else:
                    self.get_logger().warn('path_A not found in YAML.')

            elif command == 'path_B':
                if 'path_B' in self.available_paths:
                    self.selected_path_name = 'path_B'
                    self.selected_path_points = self.available_paths['path_B']
                    self.auto_phase = 'PENDING'
                    self.get_logger().info(f'Selected path_B ({len(self.selected_path_points)} pts). Ready to AUTO_START.')
                else:
                    self.get_logger().warn('path_B not found in YAML.')

            # 자율주행 시작
            elif command == 'AUTO_START':
                if not self.selected_path_points:
                    self.get_logger().warn('No path selected. SELECT Path')
                else:
                    self._publish_path(self.selected_path_name, self.selected_path_points)
                    self.auto_phase = 'RUNNING'
                    self.get_logger().info('AUTO started. Path published to /ppc/path.')
                    self.ppc_enable_pub.publish(Bool(data=True))  # PPC 활성화

            # 수동 종료
            elif command == 'DONE':
                self.state = 'IDLE'
                self.auto_phase = 'PENDING'
                self.selected_path_name = None
                self.selected_path_points = None
                self.get_logger().info('AUTO → IDLE (DONE).')
                self.ppc_enable_pub.publish(Bool(data=False))  # PPC 비활성화

            else:
                # 그대로 두되, 알림만
                self.get_logger().info(f'AUTO({self.auto_phase}) received: "{command}"')

        
        # CALIBRATION 상태일 때
        elif self.state == 'CALIBRATION':
            self.get_logger().info('State: CALIBRATION', throttle_duration_sec=1.0)
            if command == 'DONE':
                self.state = 'IDLE'

        # cmd_vel 토픽에서 수동 개입이 감지되면 MANUAL 상태로 전환
    # 3-2. 수동 개입 감지 콜백 함수
    def manual_override_callback(self, msg):
        if self.state in ['AUTO', 'CALIBRATION']:
            self.manual_override_detected = True

    
    # ========================================================================= #
    # 4. 상태 머신 루프 및 기타 함수 정의
    # ========================================================================= #
    # 4-1. 로봇 멈추는 함수 현재는 self.zero_twist 퍼블리시로 대체
    def stop_robot(self):
        # 주행체 멈추는 코드
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        # self.cmd_vel_publisher.publish(stop_msg)  # 사용하지 않음: 정지는 /cmd_vel_stop 사용

    # 4-2. 상태 머신 루프 함수
    def state_machine_loop(self):
        # AUTO 또는 CALIBRATION 상태에서 수동 개입이 감지되면 MANUAL 상태로 전환
        if self.manual_override_detected:
            if self.state == 'AUTO' or self.state == 'CALIBRATION':
                self.get_logger().warn('Manual override detected! Switching to MANUAL mode.')
                self.state = 'MANUAL'
                self.ppc_enable_pub.publish(Bool(data=False))  # PPC 비활성화
            self.manual_override_detected = False  # 플래그 리셋
            return

        # IDLE 상태에서는 최우선 정지 채널로 0 명령을 계속 퍼블리시
        if self.state == 'IDLE':
            self.stop_pub.publish(self.zero_twist)
            self.get_logger().info('State: IDLE', throttle_duration_sec=1.0)
        elif self.state == 'MANUAL':
            self.get_logger().info('State: MANUAL', throttle_duration_sec=1.0)
        elif self.state == 'AUTO':
            if self.auto_phase == 'PENDING':
                if not self.selected_path_name:
                    self.get_logger().info('AUTO mode: No path selected. Waiting for path selection.', throttle_duration_sec=5.0)
                else:
                    self.get_logger().info(f'AUTO mode: Path "{self.selected_path_name}" selected. Waiting for AUTO_START command.', throttle_duration_sec=5.0)
            else: # RUNNING
                self.get_logger().info(f'State: AUTO (RUNNING) {self.selected_path_name}', throttle_duration_sec=1.0)

            
        elif self.state == 'CALIBRATION':
            self.get_logger().info('State: CALIBRATION', throttle_duration_sec=1.0)
        else:
            self.get_logger().error(f'Unknown state: {self.state}')
            self.state = 'IDLE'
    # 4-3. 경로 퍼블리시 함수
    def _publish_path(self, path_name, points_xy):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # 전역 프레임과 일치

        poses = []
        for xy in points_xy:
            x, y = float(xy[0]), float(xy[1])
            ps = PoseStamped()
            ps.header.stamp = path_msg.header.stamp
            ps.header.frame_id = path_msg.header.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0  # 나머지 0
            poses.append(ps)

        path_msg.poses = poses
        self.ppc_path_pub.publish(path_msg)
        self.get_logger().info(f"Published '{path_name}' to /ppc/path : {len(poses)} poses (frame=map).")



def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
