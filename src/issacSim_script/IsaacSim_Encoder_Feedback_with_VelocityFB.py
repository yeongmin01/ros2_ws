import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.utils.extensions import enable_extension
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int16MultiArray, Float32MultiArray, UInt16MultiArray
import threading
import omni.kit.app
import carb.input
import omni.appwindow
import math
from omni.debugdraw import get_debug_draw_interface
from pxr import Gf
import omni.usd
from pxr import UsdPhysics
from pxr import Gf, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control

# 설정값
MAX_VEL_DEG = 300.0

# 확장 기능 활성화
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.debugdraw")
enable_extension("omni.isaac.dynamic_control")


class AGVAllWheelVisualizer(Node):
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        super().__init__("agv_all_wheel_visualizer")

        self.init_count = 0
        self.init_ignore_frames = 30  

        # 시각화 인터페이스
        self.draw = get_debug_draw_interface()
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        self.cmd_vel = 0.0
        self.cmd_steer = 0.0

        # 모드 설정 (keyboard / ros)
        self.mode = "ros"

        # 8륜 좌표 설정
        self.wheel_coords = {
            "w1_l": (4.65, 1.2665),
            "w1_r": (4.65, -1.2665),
            "w2_l": (3.0, 1.2665),
            "w2_r": (3.0, -1.2665),
            "w3_l": (-3.0, 1.2665),
            "w3_r": (-3.0, -1.2665),
            "w4_l": (-4.65, 1.2665),
            "w4_r": (-4.65, -1.2665),
        }

        # 축별/좌우 독립 조향각 하드웨어 제한
        self.wheel_steer_limits = {
            "w1_l": {"min": -33.0, "max": 27.4},
            "w1_r": {"min": -27.4, "max": 33.0},
            "w2_l": {"min": -30.0, "max": 23.0},
            "w2_r": {"min": -23.0, "max": 30.0},
            "w3_l": {"min": -23.0, "max": 30.0},
            "w3_r": {"min": -30.0, "max": 23.0},
            "w4_l": {"min": -27.4, "max": 33.0},
            "w4_r": {"min": -33.0, "max": 27.4},
        }

        self.max_steer_deg = 30.0
        self.min_radius = 9.4

        self.drive_sub = self.create_subscription(
            Float64, "/cmd_vel_linear", self.drive_cb, 10
        )
        self.steer_sub = self.create_subscription(
            Float64, "/cmd_steer_angle", self.steer_cb, 10
        )

        # 새 ROS 토픽 (4축 steering, 2축 velocity)
        self.ros_steering = [0.0, 0.0, 0.0, 0.0]
        self.ros_velocity = [0.0, 0.0]
        self.steering_cmd_sub = self.create_subscription(
            Int16MultiArray, "/vehicle/steering_cmd", self.steering_cmd_cb, 10
        )
        self.velocity_cmd_sub = self.create_subscription(
            Int16MultiArray, "/vehicle/velocity_cmd", self.velocity_cmd_cb, 10
        )

        # VCU-NCU 프로토콜 기반 조향각 피드백 Publisher
        self.steering_fb_1_pub = self.create_publisher(
            Int16MultiArray, "/Isaac/steering_fb_1", 10
        )
        self.steering_fb_2_pub = self.create_publisher(
            Int16MultiArray, "/Isaac/steering_fb_2", 10
        )

        # [수정] 속도 피드백 Publisher - 2축 선속도 (m/s)
        self.velocity_fb_pub = self.create_publisher(
            Int16MultiArray, "/Isaac/velocity_fb", 10
        )

        # 속도 피드백 Publisher (cumulative pulse, unsigned 16-bit)
        self.velocity_pulse_pub = self.create_publisher(
            UInt16MultiArray, "/Isaac/velocity_pulse", 10
        )

        # 누적 펄스 카운터 (4개 바퀴: w2_l, w2_r, w3_l, w3_r)
        self.pulse_counter = [0, 0, 0, 0]
        self.wheel_names_velocity = ["w2_l", "w2_r", "w3_l", "w3_r"]
        self.prev_angles = [None, None, None, None]
        self.prev_time = None
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self.on_update)
        )
        self.get_logger().info(
            "🚀 All-Wheel ICR Visualizer Active (with Velocity Feedback)"
        )

    def drive_cb(self, msg):
        self.cmd_vel = msg.data

    def steer_cb(self, msg):
        self.cmd_steer = msg.data

    def steering_cmd_cb(self, msg):
        # VCU-NCU 프로토콜: 300 = 30.0°
        self.ros_steering = [v / 10.0 for v in msg.data]

    def velocity_cmd_cb(self, msg):
        self.ros_velocity = [v / 100.0 / 0.7 * 180 / 3.14 for v in msg.data]

    def ncu_command_cb(self, msg):
        data = list(msg.data)

        stop_request = int(data[6]) if len(data) > 6 else 0
        align_cmd = int(data[7]) if len(data) > 7 else 0

        if align_cmd:
            self.get_logger().info("NCU: Align command received")

        if stop_request:
            self.ros_steering = [0.0, 0.0, 0.0, 0.0]
            self.ros_velocity = [0.0, 0.0]
            self.get_logger().info("NCU: Stop request - stopping AGV")
            return

        if len(data) >= 4:
            self.ros_steering = [
                float(data[0]),
                float(data[1]),
                float(data[2]),
                float(data[3]),
            ]

        if len(data) >= 6:
            self.ros_velocity = [float(data[4]), float(data[5])]

        if self.mode == "keyboard":
            self.mode = "ros"
            self.get_logger().info("NCU: Auto-switched to ROS mode")

    def clamp_steering_wheel_range(self, angle, wheel_name):
        """축별/좌우 독립 조향각 하드웨어 제한"""
        limits = self.wheel_steer_limits.get(wheel_name)
        if limits is None:
            return angle
        return max(limits["min"], min(limits["max"], angle))

    def calculate_ackermann(self, radius, velocity):
        steer_dict = {}
        vel_dict = {}
        for name, (x, y) in self.wheel_coords.items():
            if radius > 0:
                angle = math.degrees(math.atan2(x, abs(radius) - y))
            else:
                angle = math.degrees(math.atan2(x, abs(radius) + y))

            steer_dict[name] = angle if radius > 0 else -angle
            vel_dict[name] = velocity

        for name in steer_dict:
            steer_dict[name] = self.clamp_steering_wheel_range(steer_dict[name], name)

        return steer_dict, vel_dict

    def get_steering_feedback(self):
        """조향각 피드백 읽기 (x10, 반올림)"""
        fb_1, fb_2 = [], []

        # 순서 변경: 1r, 1l, 2l, 2r
        w_fb1 = ["w1_r", "w1_l", "w2_l", "w2_r"]
        w_fb2 = ["w3_r", "w3_l", "w4_l", "w4_r"]

        for name in w_fb1:
            fb_1.append(round(self._read_wheel_angle(name) * 10))
        for name in w_fb2:
            fb_2.append(round(self._read_wheel_angle(name) * 10))

        return fb_1, fb_2

    def get_velocity_feedback(self):
        """각 축별 선속도 피드백 (m/s x100, 반올림) - 2개 (2축 평균, 3축 평균)"""
        w_axle2 = ["w2_l", "w2_r"]
        w_axle3 = ["w3_l", "w3_r"]

        axle2_velocities = [self._read_wheel_linear_velocity(name) for name in w_axle2]
        axle3_velocities = [self._read_wheel_linear_velocity(name) for name in w_axle3]

        axle2_avg = round((sum(axle2_velocities) / len(axle2_velocities)) * 100)
        axle3_avg = round((sum(axle3_velocities) / len(axle3_velocities)) * 100)

        return [axle2_avg], [axle3_avg]

    def get_velocity_pulse_feedback(self):
        """각 바퀴별 누적 펄스 (unsigned 16-bit) - 4개 (2축 L/R, 3축 L/R)

        drive joint의 회전량을累积하여 펄스로 변환
        128 pulse = 1 rev (모터/ drive joint)
        """
        PULSE_PER_REV = 128.0
        current_time = self.get_clock().now()

        for i, wheel_name in enumerate(self.wheel_names_velocity):
            current_angle = self._read_drive_joint_angle(wheel_name)

            if self.prev_angles[i] is not None and self.prev_time is not None:
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                if dt > 0:
                    angle_diff = self.prev_angles[i] - current_angle

                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi

                    rev_diff = angle_diff / (2 * math.pi)
                    pulse_diff = int(rev_diff * PULSE_PER_REV)

                    self.pulse_counter[i] = (self.pulse_counter[i] + pulse_diff) % 65536

            self.prev_angles[i] = current_angle

        self.prev_time = current_time

        return self.pulse_counter

    def _read_drive_joint_angle(self, wheel_name):
        """Drive joint의 현재 회전각 읽기 (radian)"""
        try:
            num, side = wheel_name[1], wheel_name[3]
            joint_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/joints/{wheel_name}_drive_joint"

            stage = omni.usd.get_context().get_stage()
            joint_prim = stage.GetPrimAtPath(joint_path)

            if joint_prim.IsValid():
                current_pos = joint_prim.GetAttribute(
                    "state:angular:physics:position"
                ).Get()

                if current_pos is not None:
                    return current_pos

            return 0.0
        except Exception:
            return 0.0

    def _read_wheel_angle(self, wheel_name):
        """Steer joint의 현재 조향각 읽기 (radian)"""
        try:
            num, side = wheel_name[1], wheel_name[3]
            joint_path = (
                f"/World/pagv_51_reduction_260323/agv_precision_alignment/joints/w{num}_{side}_steer_joint"
            )

            stage = omni.usd.get_context().get_stage()
            joint_prim = stage.GetPrimAtPath(joint_path)

            if joint_prim.IsValid():
                current_pos = joint_prim.GetAttribute(
                    "state:angular:physics:position"
                ).Get()

                if current_pos is not None:
                    return current_pos

            return 0.0
        except Exception:
            return 0.0

    def _read_wheel_linear_velocity(self, wheel_name):
        """Wheel link의 world 선속도 읽기 (m/s) - 부호 포함

        Dynamic Control API 사용
        부호: 직진=양수, 후진=음수
        """
        try:

            if self.init_count < self.init_ignore_frames:
                self.init_count += 1
                return 0.0
            
            num, side = wheel_name[1], wheel_name[3]
            # Rigid body path: steer_link
            body_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/{wheel_name}_steer_link"

            # Dynamic Control API로 rigid body 핸들 가져오기
            body_handle = self.dc.get_rigid_body(body_path)

            if body_handle == _dynamic_control.INVALID_HANDLE:
                return 0.0

            vel = self.dc.get_rigid_body_linear_velocity(body_handle)
            
            if vel is None:
                return 0.0
            
            body_orientation_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/base_link"
            body_orientation_handle = self.dc.get_rigid_body(body_orientation_path)

            if body_orientation_handle == _dynamic_control.INVALID_HANDLE:
                return 0.0

            pose = self.dc.get_rigid_body_pose(body_orientation_handle)
            quat = pose.r  # quaternion (x, y, z, w)

            qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w

            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
                )
            yaw = yaw - math.pi/2
            heading_x = math.cos(yaw)
            heading_y = math.sin(yaw)
            #print(yaw * 180/math.pi)
            signed_vel = vel.x * heading_x + vel.y * heading_y

            #선속도 리턴
            return vel.x  #signed_vel

        except Exception:
            return 0.0

    def _get_angle_from_transform(self, prim):
        """World transform에서 Z축 회전각 계산 ( Degrees)"""
        world_matrix = omni.usd.get_world_transform_matrix(prim)
        rotation = world_matrix.ExtractRotation()
        quat = rotation.GetQuat()
        quat_gf = Gf.Quatd(quat.GetReal(), quat.GetImaginary())
        rot_gf = Gf.Rotation(quat_gf)
        return rot_gf.GetAngle()

    def on_update(self, e):
        rclpy.spin_once(self, timeout_sec=0.1)
        appw = omni.appwindow.get_default_app_window()
        kb = appw.get_keyboard()
        ii = carb.input.acquire_input_interface()

        # M 키로 모드 전환
        if ii.get_keyboard_value(kb, carb.input.KeyboardInput.M):
            if not getattr(self, "_m_key_pressed", False):
                self.mode = "ros" if self.mode == "keyboard" else "keyboard"
                self.get_logger().info(f"🔄 Mode switched to: {self.mode}")
                self._m_key_pressed = True
        else:
            self._m_key_pressed = False

        target_steers = {name: self.cmd_steer for name in self.wheel_coords}
        target_vels = {name: self.cmd_vel for name in self.wheel_coords}

        if self.mode == "ros":
            wheel_names = [
                "w1_l",
                "w1_r",
                "w2_l",
                "w2_r",
                "w3_l",
                "w3_r",
                "w4_l",
                "w4_r",
            ]
            for i, name in enumerate(wheel_names):
                steer_idx = i // 2
                if steer_idx < len(self.ros_steering):
                    raw_angle = self.ros_steering[steer_idx]
                    target_steers[name] = self.clamp_steering_wheel_range(
                        raw_angle, name
                    )
            for i, name in enumerate(wheel_names):
                vel_idx = 0 if i < 4 else 1
                if vel_idx < len(self.ros_velocity):
                    target_vels[name] = self.ros_velocity[vel_idx]
        else:
            radius, vel = 0, 0
            if ii.get_keyboard_value(kb, carb.input.KeyboardInput.W):
                vel = MAX_VEL_DEG
            elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.S):
                vel = -MAX_VEL_DEG

            if ii.get_keyboard_value(kb, carb.input.KeyboardInput.Q):
                radius, vel = self.min_radius, MAX_VEL_DEG
            elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.E):
                radius, vel = -self.min_radius, MAX_VEL_DEG
            elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.Z):
                radius, vel = self.min_radius, -MAX_VEL_DEG
            elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.C):
                radius, vel = -self.min_radius, -MAX_VEL_DEG
            elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.A):
                for n in target_steers:
                    target_steers[n] = self.clamp_steering_wheel_range(
                        self.max_steer_deg, n
                    )
            elif ii.get_keyboard_value(kb, carb.input.KeyboardInput.D):
                for n in target_steers:
                    target_steers[n] = self.clamp_steering_wheel_range(
                        -self.max_steer_deg, n
                    )
            if radius != 0:
                target_steers, target_vels = self.calculate_ackermann(radius, vel)
            elif vel != 0:
                for n in target_vels:
                    target_vels[n] = vel

        stage = stage_utils.get_current_stage()
        #self.draw.clear_lines()

        for name, (x_loc, y_loc) in self.wheel_coords.items():
            num = name[1]  # '1', '2', '3', '4'
            side = name[3]  # 'l', 'r'
            d_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/joints/w{num}_{side}_drive_joint"
            s_path = (
                f"/World/pagv_51_reduction_260323/agv_precision_alignment/joints/w{num}_{side}_steer_joint"
            )

            l_path = f"/World/pagv_51_reduction_260323/agv_precision_alignment/{name}_steer_link"

            d_prim = stage.GetPrimAtPath(d_path)
            s_prim = stage.GetPrimAtPath(s_path)
            l_prim = stage.GetPrimAtPath(l_path)

            # 1. 물리 제어 적용
            if d_prim.IsValid():
                d_prim.GetAttribute("drive:angular:physics:targetVelocity").Set(
                    target_vels[name]
                )
            if s_prim.IsValid():
                target_deg = target_steers[name]
                target_rad = math.radians(target_deg)

                s_prim.GetAttribute("drive:angular:physics:targetPosition").Set(
                    target_deg
                )
                s_prim.GetAttribute("drive:angular:physics:stiffness").Set(1e9)

            # 2. 실시간 법선 시각화 (모든 바퀴)
            if l_prim.IsValid():
                world_matrix = omni.usd.get_world_transform_matrix(l_prim)
                wheel_pos = world_matrix.ExtractTranslation()
                wheel_rot = world_matrix.ExtractRotation()

                # URDF 기준 바퀴 전방 벡터 (Local Y)
                local_forward = Gf.Vec3d(0, 1, 0)
                world_forward = wheel_rot.TransformDir(local_forward)
                world_forward.Normalize()

                # 조향 방향에 수직인 법선 벡터 계산 (ICR 방향)
                world_normal = Gf.Vec3d(-world_forward[1], world_forward[0], 0.0)
                world_normal.Normalize()

                # 양방향 선 그리기 좌표 계산
                start = wheel_pos + Gf.Vec3d(0, 0, 0.2)
                end_pos = start + world_normal * 20.0
                end_neg = start - world_normal * 20.0

                # 시각화: 노란색(안쪽)과 빨간색(바깥쪽)으로 구분하여 투사
                #self.draw.draw_lines(
                #    [tuple(start)], [tuple(end_pos)], [(1, 1, 0, 1)], [2.0]
                #)
                #self.draw.draw_lines(
                #    [tuple(start)], [tuple(end_neg)], [(1, 0, 0, 1)], [2.0]
                #)

        # 피드백 publish (각도 ° + 선속도 m/s + pulse)
        steering_fb_1, steering_fb_2 = self.get_steering_feedback()
        velocity_fb_1, velocity_fb_2 = self.get_velocity_feedback()
        velocity_pulse = self.get_velocity_pulse_feedback()

        self.steering_fb_1_pub.publish(Int16MultiArray(data=steering_fb_1))
        self.steering_fb_2_pub.publish(Int16MultiArray(data=steering_fb_2))
        # 2축 속도 피드백 (fd1, fd2) - m/s
        self.velocity_fb_pub.publish(
            Int16MultiArray(data=velocity_fb_1 + velocity_fb_2)
        )
        # 4개 바퀴 속도 피드백 (2축L, 2축R, 3축L, 3축R) - pulse
        self.velocity_pulse_pub.publish(UInt16MultiArray(data=velocity_pulse))

node = AGVAllWheelVisualizer()
