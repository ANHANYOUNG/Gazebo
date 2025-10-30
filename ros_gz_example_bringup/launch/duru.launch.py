import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # 패키지 경로
    pkg_bringup      = get_package_share_directory('ros_gz_example_bringup')
    pkg_gazebo       = get_package_share_directory('ros_gz_example_gazebo')
    pkg_description  = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')
    pkg_application  = get_package_share_directory('ros_gz_example_application')

    # world / model / config 파일 경로
    world_file = os.path.join(pkg_gazebo, 'worlds', 'agriculture_v2.sdf')
    sdf_file   = os.path.join(pkg_description, 'models', 'tracked_v2', 'tracked_v2.sdf')
    bridge_yaml = os.path.join(pkg_bringup, 'config', 'ros_gz_example_bridge.yaml')
    twist_mux_yaml = os.path.join(pkg_bringup, 'config', 'twist_mux.yaml')
    path_yaml  = os.path.join(pkg_bringup, 'config', 'path.yaml')

    # SDF 로봇 기술서 로드 (robot_state_publisher 입력)
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # terminal 런치
    with_terms_arg = DeclareLaunchArgument(
        'with_terminals',
        default_value='true',
        description='open terminals for teleop and state command'
    )
    with_terms = LaunchConfiguration('with_terminals')

    # Gazebo (Ignition/GZ) 실행
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([world_file])
        }.items(),
    )

    # robot_state_publisher (시뮬 시간 사용)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
    )

    # ROS ↔ GZ 브리지 (브리지 설정 파일 사용)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_yaml,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    # 정적 TF (센서 프레임 고정)
    static_base_prefix_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_prefix",
        arguments=["0", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link"],
        output="screen",
    )
    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link/imu_sensor"],
        output="screen",
    )
    static_gps_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gps_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "chassis_link", "tracked_v2/chassis_link/navsat_sensor"],
        output="screen",
    )

    # 로컬/글로벌 EKF (전역 좌표 /odometry/global 제공)
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')],
    )
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg_bringup, "config", "ekf.yaml"),
                    {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')],
    )

    # twist_mux (우선순위: stop > teleop > auto)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml],
    )

    # State Manager — path.yaml 로드 + 모드 관리 + /ppc/path 퍼블리시
    state_machine_node = Node(
        package='ros_gz_example_application',
        executable='state_machine.py',
        name='state_manager_node',
        output='screen',
        parameters=[path_yaml, {'use_sim_time': True}],
    )

    # Pure Pursuit 컨트롤러 — /ppc/path 구독, /cmd_vel_ppc 퍼블리시
    pure_pursuit_node = Node(
        package='ros_gz_example_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # terminal 실행 (텔레옵 및 상태 명령용)
        # 공통: 새 터미널에서 ROS 환경 다시 source
    bash_setup = (
        'bash -lc "'
        'source /opt/ros/jazzy/setup.bash; '
        'source install/setup.bash; '
        # 여기에 실제 실행 커맨드가 이어 붙음
    )

    # ① 키보드 teleop
    teleop_term = ExecuteProcess(
        condition=IfCondition(with_terms),
        cmd=[
            'gnome-terminal', '--',
            'bash', '-lc',
            # q 키로 종료, 안내 문구도 나오게
            'source /opt/ros/jazzy/setup.bash; '
            'source install/setup.bash; '
            'echo -e \'[teleop] w/s=전후, a/d=회전, q=종료\\n\'; '
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard'
        ],
        output='screen'
    )

    # ② 상태 커맨드 퍼블리셔
    state_cmd_term = ExecuteProcess(
        condition=IfCondition(with_terms),
        cmd=[
            'gnome-terminal', '--',
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash; '
            'source install/setup.bash; '
            'echo -e \'[state] AUTO -> path_A/path_B -> AUTO_START -> E_STOP\\nexit 로 종료\\n\'; '
            # entry point 이름이 'command' 라면 아래처럼, 파일명을 직접 실행하면 패키지 경로 따라 조정
            'ros2 run ros_gz_example_application cmd_send.py'
        ],
        output='screen'
    )


    # (옵션) 시각화/디버깅 노드
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(pkg_bringup, 'config', 'tracked_v1.rviz')],
    #     output='screen',
    # )

    return LaunchDescription([
        # Gazebo 시뮬레이터
        with_terms_arg,
        gz_sim,

        # 브리지 / 상태 발행
        bridge,
        robot_state_publisher,

        # EKF & TF
        ekf_local_node,
        ekf_global_node,
        static_base_prefix_tf,
        static_imu_tf,
        static_gps_tf,

        # 제어 선택기
        twist_mux_node,

        # 애플리케이션 노드
        state_machine_node,
        pure_pursuit_node,
        teleop_term,
        state_cmd_term,


        # RViz (원하면 주석 해제)
        # rviz,
    ])
