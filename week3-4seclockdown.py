cat << 'EOF' > ~/px4_ros2_ws/src/px4_msgs/fixed_wing_final_diamond.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleAttitudeSetpoint, OffboardControlMode, VehicleStatus, VehicleCommand, VehicleAttitude, VehicleLocalPosition, VehicleAirData
import time
import math
import numpy as np

class FixedWingFinalDiamond(Node):
    def _init_(self):
        super()._init_('fixed_wing_final_diamond')

        # QoS Profile: PX4 OUT topics are volatile (not latched)
        # CRITICAL FIX: Use VOLATILE durability instead of TRANSIENT_LOCAL
        # TRANSIENT_LOCAL can cause DDS to wait for history, blocking subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # PX4 publishes volatile messages
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Konfigürasyon ve Limitler ---
        self.declare_parameter('target_dist', 15.0)
        self.declare_parameter('cruise_speed', 12.0)
        self.declare_parameter('stall_speed', 8.0)
        self.declare_parameter('camera_fov_x', 80.0)
        self.declare_parameter('min_altitude', 10.0) # Hard Deck (Safety)
        
        # Limitler
        self.declare_parameter('max_roll_deg', 55.0)
        self.declare_parameter('max_pitch_deg', 20.0)
        # --- WEEK 3: Dönüş Yumuşatma & Takip Karar Parametreleri ek ---
        self.declare_parameter('max_track_roll_deg', 20.0)   # hedef takip için izin verilen max roll
        self.declare_parameter('roll_rate_limit_deg', 60.0)  # saniyede max roll değişimi
        self.declare_parameter('lock_time_required', 4.0)    # saniye
        self.max_track_roll = math.radians(self.get_parameter('max_track_roll_deg').value)
        self.max_roll_rate = math.radians(self.get_parameter('roll_rate_limit_deg').value)
        self.lock_time_required = self.get_parameter('lock_time_required').value

        
        # PID Kazançları
        self.declare_parameter('roll_kp', 1.8)  
        self.declare_parameter('pitch_kp', 1.2)
        self.declare_parameter('throttle_kp', 0.15)

        # Değişkenleri al
        self.target_dist = self.get_parameter('target_dist').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.stall_speed = self.get_parameter('stall_speed').value
        self.fov_rad_x = math.radians(self.get_parameter('camera_fov_x').value)
        self.min_alt = self.get_parameter('min_altitude').value
        
        self.max_roll_rad = math.radians(self.get_parameter('max_roll_deg').value)
        self.max_pitch_rad = math.radians(self.get_parameter('max_pitch_deg').value)

        # PID Init
        self.roll_pid = PID(kp=self.get_parameter('roll_kp').value, kd=0.2, imax=0.1, 
                            out_min=-self.max_roll_rad, out_max=self.max_roll_rad)
        
        self.pitch_pid = PID(kp=self.get_parameter('pitch_kp').value, kd=0.1, imax=0.1, 
                             out_min=-self.max_pitch_rad, out_max=self.max_pitch_rad)
        
        self.throttle_pid = PID(kp=self.get_parameter('throttle_kp').value, ki=0.05, imax=0.2, 
                                out_min=-0.3, out_max=0.4)

        # --- Subscribers & Publishers ---
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        # ATTITUDE SUBSCRIBER DISABLED: /fmu/out/vehicle_attitude_v1 not available - will use integrated yaw only
        # self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude_v1', self.attitude_cb, qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_cb, qos_profile)
        self.air_data_sub = self.create_subscription(VehicleAirData, '/fmu/out/vehicle_air_data_v1', self.air_data_cb, qos_profile)
        self.target_sub = self.create_subscription(Float32MultiArray, '/target/box', self.target_cb, 10)
        
        self.attitude_sp_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', qos_profile)
        # NOTE: PX4 expects OffboardControlMode on '/fmu/in/offboard_control_mode' (no _v1)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_v1', qos_profile)

        # --- State Variables ---
        self.last_time = self.get_clock().now()
        self.last_target_time = self.get_clock().now()
        self.last_target = None
        # --- WEEK 3: Target Lock State ek ---
        self.lock_start_time = None
        self.target_locked = False

        
        self.is_offboard = False
        self.is_armed = False
        
        # YAW: Integrated only (no attitude feedback available)
        # Starts at 0, will be updated via coordinated turn math when offboard active
        self.current_yaw = 0.0
        self.target_yaw_integrated = 0.0
        
        self.ground_speed = 0.0
        self.true_airspeed = 0.0
        self.current_alt = 0.0
        
        # OFFBOARD sequence control: must publish setpoints 10+ times before requesting offboard
        self.setpoint_count = 0
        self.offboard_transition_time = None
        
        # Note: periodic retry logic will attempt ARM/OFFBOARD every N cycles
        
        self.last_roll_cmd = 0.0
        self.last_pitch_cmd = 0.0

        # CRITICAL: Timer must be >= 50 Hz for reliable offboard
        # PX4 requires continuous setpoint stream at 20+ Hz minimum
        self.create_timer(1.0/50.0, self.control_loop)  # 50 Hz
        self.get_logger().info('FixedWing DIAMOND Node (Flight Ready) Started.')
        self.get_logger().info('Timer: 50 Hz (20 ms) - CRITICAL for offboard stability')
        self.get_logger().info('--- Offboard Sequence ---')
        self.get_logger().info('1. Will publish OffboardControlMode + VehicleAttitudeSetpoint continuously')
        self.get_logger().info('2. After 10+ messages, will request ARM')
        self.get_logger().info('3. After ARM, will request OFFBOARD mode')
        self.get_logger().info('4. Monitor logs for STATE transitions')

        # PX4 timestamp (use VehicleStatus.timestamp as ground truth for PX4 time)
        # Initialize with local microsecond time as fallback until PX4 provides its timestamp
        self.current_px4_timestamp = int(self.get_clock().now().nanoseconds / 1000)

    def status_cb(self, msg: VehicleStatus):
        # Track PX4 state and capture PX4 timestamp for outgoing messages
        self.is_offboard = (msg.nav_state == 14)
        self.is_armed = (msg.arming_state == 2)
        # msg.timestamp is in microseconds in PX4 messages
        try:
            self.current_px4_timestamp = int(msg.timestamp)
        except Exception:
            # If timestamp not present or invalid, keep previous value
            pass

    # DISABLED: attitude_v1 topic not available from PX4
    # Yaw estimation handled through coordinated turn integration only
    # def attitude_cb(self, msg: VehicleAttitude):
    #     q = msg.q
    #     siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    #     cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    #     current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    #     self.current_yaw = current_yaw_rad
    #     if not self.is_offboard:
    #         self.target_yaw_integrated = current_yaw_rad

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.ground_speed = math.sqrt(msg.vx*2 + msg.vy*2)
        self.current_alt = -msg.z # NED coordinate system (z is negative up)

    def air_data_cb(self, msg: VehicleAirData):
        # RISK: Airspeed can be NAN or stale on fixed-wing
        # Guard against NAN to prevent control instability
        if not math.isnan(msg.true_airspeed_m_s):
            self.true_airspeed = msg.true_airspeed_m_s
        # else: keep previous value

    def target_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self.last_target_time = self.get_clock().now()
            self.last_target = {'cx': msg.data[0], 'cy': msg.data[1], 'dist': msg.data[4]}
            # week 3 ek
            if self.lock_start_time is None:
              self.lock_start_time = now
              self.target_locked = False

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        # For SITL compatibility use 0 timestamp so PX4 accepts commands immediately
        msg.timestamp = 0
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.get_logger().info(f'COMMAND: {command}, p1={p1}, p2={p2}')
        self.vehicle_command_pub.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]

    def get_effective_speed(self):
        # Güvenilir Hız Seçimi (Hybrid)
        # Eğer Airspeed sensörü veri veriyorsa (> 3m/s) onu kullan, yoksa GPS hızı.
        if self.true_airspeed > 3.0:
            return self.true_airspeed
        return self.ground_speed

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.02  # 50 Hz nominal
        self.last_time = now

        # --- TARGET VALIDATION ---
        # time since last target and if target exists
        time_diff = (now - self.last_target_time).nanoseconds / 1e9
        target_valid = (time_diff < 1.0) and (self.last_target is not None)

        # Base throttle for fixed-wing cruise (used in target computations)
        base_throttle = 0.6

        # DEBUG: Log offboard state periodically
        if self.setpoint_count % 50 == 0:  # Every ~1 second at 50 Hz
            self.get_logger().info(
                f'STATE: armed={self.is_armed}, offboard={self.is_offboard}, '
                f'setpoint_count={self.setpoint_count}'
            )

        # --- CONTROL LOGIC (target generation) ---
        if target_valid:
            t = self.last_target
            
            # 1. FOV Tabanlı Açı Hesabı
            angle_err_x = (t['cx'] - 0.5) * self.fov_rad_x
            angle_err_y = (0.5 - t['cy']) * (self.fov_rad_x * 0.75)
            err_dist = self.target_dist - t['dist']

            # 2. PID
            target_roll = self.roll_pid.update(angle_err_x, dt)
            target_pitch = self.pitch_pid.update(angle_err_y, dt)
            # --- WEEK 3: 4s Target Lock Logic ---
            lock_elapsed = 0.0
            if self.lock_start_time is not None:
              lock_elapsed = (now - self.lock_start_time).nanoseconds / 1e9
            if lock_elapsed >= self.lock_time_required:
              self.target_locked = True
            # --- WEEK 3: takip uygunluğu ek ---
            if abs(target_roll) > self.max_track_roll:
              # İzin verilen açı aşılıyorsa takibi bırak
              self.target_locked = False
              self.lock_start_time = None
             # --- 4.3 FIX: Takibi bırakma YOK, sadece saturasyon ---
             if abs(target_roll) > self.max_track_roll:
              target_roll = math.copysign(self.max_track_roll, target_roll)

    
            # 3. Energy Mixing (TECS Emulation + Bank Compensation)
            # Pitch Up -> Throttle Up
            pitch_throttle_ff = max(0.0, target_pitch) * (0.25 / math.radians(20))
            # Bank Angle -> Throttle Up (Dönüşte irtifa kaybını önle)
            roll_throttle_ff = abs(target_roll) * (0.15 / math.radians(45))
            
            pid_thr = self.throttle_pid.update(err_dist, dt)
            target_throttle = base_throttle + pid_thr + pitch_throttle_ff + roll_throttle_ff
            # --- WEEK 3: roll sınırı ek ---
            max_delta_roll = self.max_roll_rate * dt
            roll_delta = target_roll - self.last_roll_cmd
            if abs(roll_delta) > max_delta_roll:
              target_roll = self.last_roll_cmd + math.copysign(max_delta_roll, roll_delta)


        else:
            # --- Hedef Kaybı: Smooth Decay & Loiter ---
            # --- WEEK 3: hedef kaybı ek ---
            self.lock_start_time = None
            self.target_locked = False

            if time_diff < 3.0:
                decay_factor = 0.95
                target_roll = self.last_roll_cmd * decay_factor
                # BUG FIX: last_pitch_cmd kullanıldı
                target_pitch = self.last_pitch_cmd * decay_factor 
                target_throttle = base_throttle
            else:
                # Loiter Modu:
                # Biraz daha dik dön (20 derece) ki alan dışına kaçmasın
                target_roll = math.radians(20.0) 
                target_pitch = math.radians(3.0) # Hafif tırmanış
                target_throttle = 0.65
        

        # --- SAFETY: Hız ve İrtifa Korumaları ---
        effective_speed = self.get_effective_speed()
        
        # A. Stall Guard (Rüzgar düzeltmeli)
        if effective_speed < self.stall_speed and effective_speed > 1.0:
            self.get_logger().warn(f'STALL GUARD! Spd: {effective_speed:.1f}')
            target_pitch = min(target_pitch, math.radians(-5.0)) # Burun ez
            target_throttle = 0.9  # Tam gaz (0.4-0.9 range içinde)
            target_roll = 0.0 # Kanat düzelt
            
        # B. Hard Deck (Zemin Çarpışma Önleyici)
        # Eğer irtifa limitin altındaysa ve pitch negatifse (dalıştaysa) müdahale et
        elif self.current_alt < self.min_alt and self.is_armed and self.is_offboard:
            if target_pitch < 0.0:
                self.get_logger().warn(f'HARD DECK! Alt: {self.current_alt:.1f}')
                target_pitch = math.radians(10.0) # Zorla tırmandır
                target_throttle = max(target_throttle, 0.8)

        # Apply command limits
        target_roll = max(min(target_roll, self.max_roll_rad), -self.max_roll_rad)
        target_pitch = max(min(target_pitch, self.max_pitch_rad), -self.max_pitch_rad)
        # OPTIONAL: Tighter throttle bounds (0.4-0.9) for better ESC performance
        target_throttle = max(min(target_throttle, 0.9), 0.4)

        self.last_roll_cmd = target_roll
        self.last_pitch_cmd = target_pitch

        # --- Coordinated Turn ---
        g = 9.81
        v = max(effective_speed, 6.0) # Division guard
        turn_rate = (g * math.tan(target_roll)) / v
        
        # OPTIONAL: Clamp turn rate to prevent extreme yaw changes if airspeed drops
        turn_rate = max(min(turn_rate, 0.6), -0.6)
        
        # CRITICAL: Do NOT integrate yaw without attitude feedback
        # PX4 has heading_good_for_control=false (no attitude_v1 available)
        # When offboard rejects yaw setpoint, oscillation occurs
        # SOLUTION: Keep yaw at 0.0, let bank angle do the turn (coordinated turn)
        # Bank angle naturally commands yaw via body dynamics
        self.target_yaw_integrated = 0.0
        # Set yaw_sp_move_rate to 0 for stable heading
        yaw_sp_move_rate = 0.0

        q_des = self.euler_to_quaternion(target_roll, target_pitch, self.target_yaw_integrated)

        att_msg = VehicleAttitudeSetpoint()
        # CRITICAL for SITL: use 0 as timestamp to avoid PX4 time-base rejections
        att_msg.timestamp = 0
        # CRITICAL: Field name is 'q_d' in px4_msgs for VehicleAttitudeSetpoint
        # Use q_d (float32[4]) — empty or wrong field will cause PX4 to reject setpoint
        att_msg.q_d = [
            float(q_des[0]),
            float(q_des[1]),
            float(q_des[2]),
            float(q_des[3]),
        ]
        # CRITICAL: thrust_body[0] must never be zero
        # PX4 uses this to determine if setpoint is valid
        # Clamp to [0.1, 1.0] to ensure PX4 accepts control
        safe_throttle = max(target_throttle, 0.1)
        att_msg.thrust_body = [float(safe_throttle), 0.0, 0.0]
        att_msg.yaw_sp_move_rate = float(yaw_sp_move_rate)
        
        self.attitude_sp_pub.publish(att_msg)
        
        # CRITICAL: Publish order matters - setpoint BEFORE mode
        # If mode is published before setpoint, PX4 may reject
        offboard_msg = OffboardControlMode()
        # CRITICAL for SITL: use 0 as timestamp to force PX4 to accept setpoints immediately
        offboard_msg.timestamp = 0
        # CRITICAL: Fixed-wing offboard MUST use attitude control
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True  # <-- REQUIRED for fixed-wing
        offboard_msg.body_rate = False
        # Additional OffboardControlMode fields required by px4_msgs
        offboard_msg.thrust_and_torque = False
        offboard_msg.direct_actuator = False
        self.offboard_mode_pub.publish(offboard_msg)
        self.setpoint_count += 1

        # CRITICAL: Send commands conditionally, NOT spammed every 50 cycles
        # This prevents PX4 state oscillation from repeated mode/arm requests
        # ALSO: Must send 10+ setpoints BEFORE requesting offboard mode
        # CRITICAL: Use state latch - commands are ONE-SHOT
        
        # Periodic retry: every 100 cycles (~2s at 50Hz) attempt ARM/OFFBOARD until accepted
        if self.setpoint_count % 100 == 0:
            if not self.is_armed:
                self.get_logger().info('Arming (periodic attempt)...')
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            elif self.is_armed and not self.is_offboard:
                # require at least 10 setpoints before requesting offboard
                if self.setpoint_count >= 10:
                    self.get_logger().info('Requesting OFFBOARD (periodic attempt)...')
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            else:
                self.get_logger().info('Uçuş Modu: OFFBOARD - AKTİF')

class PID:
    def _init_(self, kp, ki=0.0, kd=0.0, imax=0.0, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax = imax
        self.out_min, self.out_max = out_min, out_max
        self.integral, self.last_err = 0.0, None
    
    def update(self, err, dt):
        if dt <= 0: return 0.0
        p = self.kp * err
        self.integral = max(min(self.integral + err*dt, self.imax), -self.imax)
        d = (err - self.last_err) / dt if self.last_err is not None else 0.0
        self.last_err = err
        return max(min(p + self.ki*self.integral + self.kd*d, self.out_max), self.out_min)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingFinalDiamond()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
EOF

