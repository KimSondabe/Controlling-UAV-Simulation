import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'my_robot_controller'

# ---- Đường dẫn: đổi cho đúng máy bạn ----
QGC_PATH = os.environ.get('QGC_PATH', os.path.expanduser('~/QGroundControl.AppImage'))
RVIZ_CFG = '/home/kimson/ros2_ws/src/my_robot_controller/my_robot_controller/drone.rviz'

# ---- Bố cục cửa sổ (pixel). Đổi SW, SH nếu màn hình không phải 1920x1080 ----
SW, SH = 1920, 1080
TOP = 30                      # chừa thanh trên cùng của Ubuntu
HALF = SW // 2

LAYOUT = [
    # (chuỗi có trong tiêu đề cửa sổ, x, y, rộng, cao)
    ('Gazebo Sim',     0,    TOP,  HALF, 740),   # trái, phần trên
    ('PX4 SITL',       0,    780,  HALF, 290),   # trái, phần dưới
    ('RViz',           HALF, TOP,  HALF, 400),   # phải, trên
    ('control_node',   HALF, 440,  HALF, 300),   # phải, giữa
    ('QGroundControl', HALF, 750,  HALF, 320),   # phải, dưới
]


def place_script():
    """Mỗi cửa sổ có một vòng chờ riêng: đợi nó xuất hiện (tối đa ~3 phút) rồi đặt đúng chỗ."""
    parts = []
    for title, x, y, w, h in LAYOUT:
        parts.append(
            f'( for i in $(seq 1 90); do '
            f'if wmctrl -l | grep -qi "{title}"; then '
            f'sleep 2; '
            f'wmctrl -r "{title}" -b remove,maximized_vert,maximized_horz; '
            f'wmctrl -r "{title}" -e 0,{x},{y},{w},{h}; '
            f'break; fi; sleep 2; done ) &'
        )
    return '\n'.join(parts) + '\nwait'


def generate_launch_description():
    # ---- Tham số đổi được từ dòng lệnh, ví dụ: px4_target:=gz_x500_walls ----
    args = [
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'),
                              description='Thư mục PX4-Autopilot'),
        DeclareLaunchArgument('px4_target', default_value='gz_x500',
                              description='Target make: gz_x500, gz_x500_walls, ...'),
        DeclareLaunchArgument('ws_dir', default_value=os.path.expanduser('~/ros2_ws'),
                              description='Workspace ROS 2'),
        DeclareLaunchArgument('odom_topic', default_value='/fmu/out/vehicle_odometry',
                              description='Topic odometry của PX4'),
        DeclareLaunchArgument('world', default_value='custom',
                              description='Tên world Gazebo (trùng tên file .sdf, không có đuôi)'),
    ]
    px4_dir = LaunchConfiguration('px4_dir')
    px4_target = LaunchConfiguration('px4_target')
    ws_dir = LaunchConfiguration('ws_dir')

    # ---- 1. Cầu nối PX4 <-> ROS 2 (tắt agent cũ nếu còn, tránh trùng cổng 8888) ----
    agent = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -x MicroXRCEAgent; sleep 1; exec MicroXRCEAgent udp4 -p 8888'],
        name='micro_xrce_agent', output='screen', emulate_tty=True)

    # ---- 2. PX4 SITL + Gazebo (terminal riêng, có shell pxh) ----
        # ---- 2. PX4 SITL + Gazebo (terminal riêng, có shell pxh) ----
    px4 = ExecuteProcess(
        cmd=['gnome-terminal', '--title=PX4 SITL', '--', 'bash', '-c',
             ['cd ', px4_dir, ' && PX4_GZ_WORLD=', LaunchConfiguration('world'),
              ' make px4_sitl ', px4_target, '; exec bash']],
        name='px4_sitl')

    # ---- 3. QGroundControl (chỉ chạy nếu tìm thấy file) ----
    qgc = []
    if os.path.exists(QGC_PATH):
        qgc = [ExecuteProcess(cmd=[QGC_PATH], name='qgroundcontrol', output='log')]
    else:
        print(f'[drone_viz] Không thấy QGroundControl tại {QGC_PATH}, bỏ qua')

    # ---- 4. Node chuyển odometry PX4 -> /odom, TF, /drone/path ----
    odom = Node(
        package=PKG, executable='px4_to_odom', name='px4_to_odom', output='screen',
        parameters=[{'odom_topic': LaunchConfiguration('odom_topic')}])

    # ---- 5. RViz2 nạp file cấu hình (nếu không thấy file thì mở với cấu hình trống) ----
    if not os.path.exists(RVIZ_CFG):
        print(f'[drone_viz] Không thấy file RViz tại {RVIZ_CFG}')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', RVIZ_CFG] if os.path.exists(RVIZ_CFG) else [])

    # ---- 6. control_node: cần terminal riêng để đọc bàn phím ----
    control = ExecuteProcess(
        cmd=['gnome-terminal', '--title=control_node', '--', 'bash', '-c',
             ['source /opt/ros/humble/setup.bash && source ', ws_dir,
              '/install/setup.bash && ros2 run ', PKG, ' control_node; exec bash']],
        name='control_node')

    # ---- 7. Sắp xếp cửa sổ theo bố cục LAYOUT (chỉ chạy được trên Xorg + wmctrl) ----
    arrange = ExecuteProcess(cmd=['bash', '-c', place_script()], name='arrange_windows')

    return LaunchDescription(args + qgc + [
        agent,
        TimerAction(period=3.0, actions=[px4]),
        odom,
        rviz,
        TimerAction(period=25.0, actions=[control]),
        arrange,
    ])