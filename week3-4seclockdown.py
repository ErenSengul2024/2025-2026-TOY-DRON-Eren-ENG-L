cat << 'EOF' > /root/px4_ros2_ws/src/gok/gok/fixed_wing_final_diamond.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleAttitudeSetpoint, OffboardControlMode, VehicleStatus, VehicleCommand, VehicleAttitude, VehicleLocalPosition, VehicleAirData, VehicleGlobalPosition, TrajectorySetpoint
import time
import math
import numpy as np

class FixedWingFinalDiamond(Node):
    def _init_(self):
        super()._init_('fixed_wing_final_diamond')
        self.now = self.get_clock().now()
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
        self.declare_parameter('target_dist', 80.0)  # GPS takip mesafesi (metre)
        self.declare_parameter('cruise_speed', 12.0)
        self.declare_parameter('stall_speed', 8.0)
        self.declare_parameter('camera_fov_x', 60.0)  # Sanal kamera FOV (derece)
        self.declare_parameter('min_altitude', 10.0)  # Hard Deck (Safety)
        
        # Limitler
        self.declare_parameter('max_roll_deg', 55.0)
        self.declare_parameter('max_pitch_deg', 20.0)
        # --- WEEK 3: Dönüş Yumuşatma & Takip Karar Parametreleri ek ---
        self.declare_parameter('max_track_roll_deg', 20.0)   # hedef takip için izin verilen max roll
        self.declare_parameter('roll_rate_limit_deg', 60.0)  # saniyede max roll değişimi
        self.declare_parameter('lock_time_required', 4.0)    # saniye
        self.declare_parameter('gps_track_mode', True)       # GPS modunu aç/kapat
        
        self.max_track_roll = math.radians(self.get_parameter('max_track_roll_deg').value)
        self.max_roll_rate = math.radians(self.get_parameter('roll_rate_limit_deg').value)
        self.lock_time_required = self.get_parameter('lock_time_required').value
        self.gps_track_mode = self.get_parameter('gps_track_mode').value

        
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

        # --- Subscribers & Publishers (NAMESPACE: /px4_1/) ---
        self.status_sub = self.create_subscription(VehicleStatus, '/px4_1/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        # ATTITUDE SUBSCRIBER: Heading (yaw) için gerekli
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/px4_1/fmu/out/vehicle_attitude_v1', self.attitude_cb, qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position_v1', self.local_pos_cb, qos_profile)
        self.air_data_sub = self.create_subscription(VehicleAirData, '/px4_1/fmu/out/vehicle_air_data_v1', self.air_data_cb, qos_profile)
        self.target_sub = self.create_subscription(Float32MultiArray, '/target/box', self.target_cb, 10)
        # ✅ GPS LEADER SUBSCRIPTION: Lider uçaktan (px4_2 namespace)
        self.leader_gps_sub = self.create_subscription(VehicleGlobalPosition, '/px4_2/fmu/out/vehicle_global_position', self.leader_gps_cb, qos_profile)
        # ✅ FIX: OWN GLOBAL POSITION (GPS takip için KRITIK)
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, '/px4_1/fmu/out/vehicle_global_position', self.global_pos_cb, qos_profile)
        
        # --- PUBLISHERS (NAMESPACE: /px4_1/) ---
        self.attitude_sp_pub = self.create_publisher(VehicleAttitudeSetpoint, '/px4_1/fmu/in/vehicle_attitude_setpoint_v1', qos_profile)
        # ✅ FIX: TRAJECTORY SETPOINT PUBLISHER (PX4 FW ister velocity)
        self.traj_sp_pub = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)
        # NOTE: PX4 expects OffboardControlMode on '/px4_1/fmu/in/offboard_control_mode' (no _v1)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command_v1', qos_profile)

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
        
        # --- GPS TRACKING STATE (GPS → sanal box) ---
        self.own_lat = 0.0
        self.own_lon = 0.0
        self.own_alt = 0.0
        self.own_heading = 0.0  # radians
        
        # ✅ ALIAS: self.lat ve self.lon (debug için)
        self.lat = 0.0
        self.lon = 0.0
        
        self.leader_lat = None  # Leader uçağın GPS (harici kaynak)
        self.leader_lon = None
        self.leader_alt = None
        self.leader_gps_age = float('inf')  # Veri yaşı (saniye)
        
        self.gps_err_x = 0.0  # Bearing error → roll
        self.gps_err_y = 0.0  # Distance/altitude error → pitch
        self.gps_target_valid = False  # GPS'ten box oluşturulabilir mi?
        self.last_gps_update = self.get_clock().now()
        self.last_gps_distance = 0.0  # ✅ FIX: Mesafe takip (debug)
        
        # OFFBOARD sequence control: must publish setpoints 10+ times before requesting offboard
        self.setpoint_count = 0
        self.offboard_transition_time = None
        
        # Note: periodic retry logic will attempt ARM/OFFBOARD every N cycles
        
        self.last_roll_cmd = 0.0
        self.last_pitch_cmd = 0.0
        
        # --- OFFBOARD STATE MANAGEMENT ---
        self.arm_requested = False       # ARM komutu gönderildi mi?
        self.offboard_requested = False  # OFFBOARD komutu gönderildi mi?

        # --- FLIGHT PHASE MANAGEMENT (Trajectory ↔ Attitude) ---
        # Faze–1: Trajectory (seyir/takip)
        # Faze–2: Attitude (terminal/kamikaze/hassas manevra)
        # Faze–3: Land (iniş)
        self.flight_phase = 1  # Start at Faze–1 (trajectory)
        self.phase_enter_time = None
        self.terminal_condition_met = False
        self.terminal_engage_time = None
        
        # Terminal faze thresholds
        self.terminal_distance_threshold = 30.0  # metre (trigger at this distance)
        self.terminal_min_lock_time = 2.0  # seconds (lock stabil before entering)
        self.terminal_duration_max = 10.0  # seconds (max time in terminal phase)

        # ✅ FIX: 100 Hz timer (FW offboard için önemli)
        # PX4 FW offboard minimum 20 Hz, pratik 80-100 Hz ister
        self.create_timer(0.01, self.control_loop)  # 100 Hz
        self.get_logger().info('FixedWing DIAMOND Node (Flight Ready) Started.')
        self.get_logger().info('🔧 NAMESPACE: /px4_1 (takipçi uçak)')
        self.get_logger().info('🔗 LEADER: /px4_2/fmu/out/vehicle_global_position')
        self.get_logger().info('Timer: 100 Hz (10 ms) - CRITICAL for FW offboard stability')
        self.get_logger().info('--- Offboard Sequence ---')
        self.get_logger().info('1. Will publish OffboardControlMode + VehicleAttitudeSetpoint continuously')
        self.get_logger().info('2. After 20+ cycles, will request ARM')
        self.get_logger().info('3. After ARM, will request OFFBOARD mode (ONE-SHOT)')
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

    def attitude_cb(self, msg: VehicleAttitude):
        """Yaw (heading) hesapla quaternion'dan"""
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.own_heading = current_yaw_rad  # GPS hesabı için gerekli
        self.current_yaw = current_yaw_rad
        if not self.is_offboard:
            self.target_yaw_integrated = current_yaw_rad

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
        now = self.get_clock().now()
        if len(msg.data) >= 5:
            self.last_target_time = self.get_clock().now()
            self.last_target = {'cx': msg.data[0], 'cy': msg.data[1], 'dist': msg.data[4]}
            # week 3 ek
            if self.lock_start_time is None:
              self.lock_start_time = now
              self.target_locked = False

    def leader_gps_cb(self, msg: VehicleGlobalPosition):
        """
        Leader uçağın GPS verilerini al (VehicleGlobalPosition from /px4_2).
        
        ✅ KRITIK: VehicleGlobalPosition birimi otomatik algıla!
        - Eğer |lat| > 180 ise → 1e-7 olarak kabul et, *1e-7 yap
        - Eğer |lat| < 180 ise → zaten derece olarak kabul et (no scaling)
        
        Format:
        - lat, lon: degrees × 1e-7 VEYA zaten degrees (PX4 versiyonuna göre)
        - alt: millimeters (need to convert to meters)
        """
        try:
            # ✅ BIRIM OTOMATIK ALGILAMA
            raw_lat = msg.lat
            raw_lon = msg.lon
            raw_alt = msg.alt
            
            # Eğer |lat| > 180 ise, 1e-7 cinsinde olduğunu kabul et
            if abs(raw_lat) > 180:
                scaled_lat = raw_lat * 1e-7
            else:
                scaled_lat = raw_lat
            
            # Eğer |lon| > 180 ise, 1e-7 cinsinde olduğunu kabul et
            if abs(raw_lon) > 180:
                scaled_lon = raw_lon * 1e-7
            else:
                scaled_lon = raw_lon
            
            # Alt her zaman mm (millimeters)
            if abs(raw_alt) > 10000:  # > 10000mm = > 10km (çok yüksek olurdu)
                scaled_alt = raw_alt * 1e-3
            else:
                scaled_alt = raw_alt
            
            self.leader_lat = scaled_lat
            self.leader_lon = scaled_lon
            self.leader_alt = scaled_alt
            self.last_gps_update = self.get_clock().now()
            
            # ✅ DEBUG: Birim kontrolü ile göster
            print(f"DEBUG LEADER GPS: raw=({raw_lat}, {raw_lon}, {raw_alt}mm) → scaled=({scaled_lat:.6f}°, {scaled_lon:.6f}°, {scaled_alt:.1f}m)")
            
            if self.setpoint_count % 50 == 0:
                self.get_logger().info(
                    f'🔗 Leader GPS: lat={scaled_lat:.6f}°, lon={scaled_lon:.6f}°, alt={scaled_alt:.1f}m'
                )
        except Exception as e:
            self.get_logger().error(f'❌ Leader GPS parse failed: {e}')
            print(f"DEBUG LEADER GPS ERROR: {e}")

    def global_pos_cb(self, msg: VehicleGlobalPosition):
        """
        Own aircraft global position (lat/lon/alt) from PX4.
        
        ✅ KRITIK BIRIM ALGISI:
        - Eğer |lat| > 180 ise → 1e-7 olarak kabul et
        - Eğer |lat| < 180 ise → zaten derece
        
        VehicleGlobalPosition format:
        - lat, lon: degrees × 1e-7 VEYA degrees
        - alt: millimeters (convert to meters)
        """
        try:
            raw_lat = msg.lat
            raw_lon = msg.lon
            raw_alt = msg.alt
            
            # ✅ BIRIM OTOMATIK ALGILAMA
            # Eğer |lat| > 180 ise, 1e-7 cinsinde olduğunu kabul et
            if abs(raw_lat) > 180:
                scaled_lat = raw_lat * 1e-7
            else:
                scaled_lat = raw_lat
            
            # Eğer |lon| > 180 ise, 1e-7 cinsinde olduğunu kabul et
            if abs(raw_lon) > 180:
                scaled_lon = raw_lon * 1e-7
            else:
                scaled_lon = raw_lon
            
            # Alt her zaman mm (millimeters)
            if abs(raw_alt) > 10000:  # > 10000mm = > 10km (çok yüksek)
                scaled_alt = raw_alt * 1e-3
            else:
                scaled_alt = raw_alt
            
            self.own_lat = scaled_lat
            self.own_lon = scaled_lon
            self.own_alt = scaled_alt
            
            # ✅ ALIAS: self.lat ve self.lon (debug için)
            self.lat = scaled_lat
            self.lon = scaled_lon
            
            # ✅ DEBUG: Birim kontrolü ile göster
            if self.setpoint_count % 100 == 0:
                print(f"DEBUG OWN GPS: raw=({raw_lat}, {raw_lon}, {raw_alt}mm) → scaled=({scaled_lat:.6f}°, {scaled_lon:.6f}°, {scaled_alt:.1f}m)")
                self.get_logger().info(f'REAL GPS -> Own: {self.lat:.4f}, Leader: {self.leader_lat if self.leader_lat else 0.0:.4f}')
        except Exception as e:
            self.get_logger().error(f'❌ Own GPS parse failed: {e}')
            print(f"DEBUG OWN GPS ERROR: {e}")

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

    def wrap_pi(self, angle):
        """Açıyı [-π, π] aralığına normalize et"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def bearing_and_distance(self, lat1, lon1, lat2, lon2):
        """
        İki GPS koordinatı arasında bearing (rumba) ve mesafe hesapla.
        Haversine formülü kullanır.
        
        Args:
            lat1, lon1: Başlangıç noktası (DERECE - 0-90 arası)
            lat2, lon2: Hedef noktası (DERECE - 0-90 arası)
        
        Return:
            distance (metre)
            bearing (radians, [-π, π])
        """
        try:
            # ✅ GEÇERLILIK KONTROLÜ: Koordinatlar derece cinsinde mi?
            if abs(lat1) > 90 or abs(lon1) > 180 or abs(lat2) > 90 or abs(lon2) > 180:
                print(f"⚠️ HAVERSINE: Invalid coordinates! p1=({lat1}, {lon1}) p2=({lat2}, {lon2})")
                print(f"   Coordinates should be in degrees: lat [-90,90], lon [-180,180]")
                return 0.0, 0.0
            
            # ✅ DEBUG: Giriş değerleri
            if self.setpoint_count % 100 == 0:
                print(f"DEBUG HAVERSINE INPUT: own=({lat1:.6f}°, {lon1:.6f}°) leader=({lat2:.6f}°, {lon2:.6f}°)")
            
            R = 6371000.0  # Dünya yarıçapı (metre)
            φ1 = math.radians(lat1)
            φ2 = math.radians(lat2)
            Δφ = math.radians(lat2 - lat1)
            Δλ = math.radians(lon2 - lon1)
            
            # Haversine: Mesafe hesabı
            a = math.sin(Δφ/2)*2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)*2
            d = 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
            
            # Bearing hesabı
            y = math.sin(Δλ) * math.cos(φ2)
            x = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(Δλ)
            bearing = math.atan2(y, x)
            
            # ✅ DEBUG: Çıkış değerleri
            if self.setpoint_count % 100 == 0:
                print(f"DEBUG HAVERSINE OUTPUT: dist={d:.1f}m, bearing={math.degrees(bearing):.1f}°")
            
            return d, bearing
        except Exception as e:
            self.get_logger().error(f'❌ Bearing calc failed: {e}')
            print(f"DEBUG BEARING ERROR: {e}")
            return 0.0, 0.0

    def update_gps_target(self):
        """
        GPS'ten sanal vision box oluştur.
        Follower → Leader vektörünü headinga göre normalize et.
        """
        # ✅ FIX: Leader verisi var mı kontrol et
        if self.leader_lat is None or self.leader_lon is None:
            if self.setpoint_count % 100 == 0:
                print(f"❌ Leader GPS is NULL - subscriptions working? leader_lat={self.leader_lat}, leader_lon={self.leader_lon}")
                self.get_logger().warn('❌ Leader GPS is NULL - subscriptions working?')
            self.gps_target_valid = False
            return
        
        # Kendi verisi var mı kontrol et
        if self.own_lat == 0.0 and self.own_lon == 0.0:
            if self.setpoint_count % 100 == 0:
                print(f"❌ Own GPS is (0.0, 0.0) - /px4_1 subscription down?")
                self.get_logger().warn('❌ Own GPS is (0.0, 0.0) - /px4_1 subscription down?')
            self.gps_target_valid = False
            return
        
        # ✅ DEBUG: Koordinatların magnitude'ünü kontrol et
        if self.setpoint_count % 100 == 0:
            print(f"DEBUG UPDATE_GPS_TARGET: own=({self.own_lat:.6f}°, {self.own_lon:.6f}°), leader=({self.leader_lat:.6f}°, {self.leader_lon:.6f}°)")
            print(f"  → OWN MAGNITUDE: |lat|={abs(self.own_lat):.2f}, |lon|={abs(self.own_lon):.2f}")
            print(f"  → LEADER MAGNITUDE: |lat|={abs(self.leader_lat):.2f}, |lon|={abs(self.leader_lon):.2f}")
        
        # GPS yaşını kontrol et
        now = self.get_clock().now()
        self.leader_gps_age = (now - self.last_gps_update).nanoseconds / 1e9
        
        if self.leader_gps_age > 0.5:
            if self.setpoint_count % 100 == 0:
                print(f"⏱️ Leader GPS stale: {self.leader_gps_age:.2f}s")
            self.get_logger().warn(f'⏱️ Leader GPS stale: {self.leader_gps_age:.2f}s')
            self.gps_target_valid = False
            return
        
        # Mesafe ve bearing hesapla
        dist, bearing = self.bearing_and_distance(
            self.own_lat, self.own_lon,
            self.leader_lat, self.leader_lon
        )
        
        # ✅ DEBUG: Mesafe kontrol
        if self.setpoint_count % 100 == 0:
            print(f"DEBUG GPS CALC RESULT: dist={dist:.1f}m, bearing={math.degrees(bearing):.1f}°")
        
        # Heading'e göre göreceli açı (bearing error)
        bearing_error = self.wrap_pi(bearing - self.own_heading)
        
        # (A) err_x: Bearing → Roll
        # FOV = 60°, normalize et
        max_fov_rad = math.radians(self.get_parameter('camera_fov_x').value / 2.0)
        self.gps_err_x = bearing_error / max_fov_rad
        self.gps_err_x = max(-1.0, min(1.0, self.gps_err_x))
        
        # (B) err_y: Distance error → Pitch
        desired_dist = self.target_dist
        dist_error = dist - desired_dist
        self.gps_err_y = dist_error / 50.0  # 50m normalizasyon
        self.gps_err_y = max(-1.0, min(1.0, self.gps_err_y))
        
        self.gps_target_valid = True
        
        # ✅ FIX: Mesafe kayıt etme (debug için)
        self.last_gps_distance = dist
        
        # Debug log
        if self.setpoint_count % 50 == 0:
            self.get_logger().info(
                f'🔍 GPS UPDATE: own=({self.own_lat:.6f}°, {self.own_lon:.6f}°) leader=({self.leader_lat:.6f}°, {self.leader_lon:.6f}°) | '
                f'dist={dist:.1f}m, bearing={math.degrees(bearing):.1f}°, err_x={self.gps_err_x:.2f}, err_y={self.gps_err_y:.2f}'
            )

    def check_terminal_conditions(self, dist):
        """
        Terminal faze giriş koşullarını kontrol et.
        
        Koşullar:
        1. Mesafe < threshold
        2. Target lock stabil (≥ min_lock_time)
        3. Target valid
        """
        now = self.get_clock().now()
        
        # Koşul 1: Mesafe
        if dist >= self.terminal_distance_threshold:
            self.terminal_condition_met = False
            return False
        
        # Koşul 2: Target lock sabitliği
        if self.lock_start_time is None:
            return False
        
        lock_elapsed = (now - self.lock_start_time).nanoseconds / 1e9
        if lock_elapsed < self.terminal_min_lock_time:
            return False
        
        # Koşul 3: Target valid
        if not self.gps_target_valid:
            return False
        
        # ✅ Tüm koşullar sağlandı
        self.terminal_condition_met = True
        return True

    def enter_terminal_phase(self):
        """Terminal faze'ye gir (Faze–1 → Faze–2)"""
        now = self.get_clock().now()
        if self.flight_phase != 2:
            self.flight_phase = 2
            self.phase_enter_time = now
            self.terminal_engage_time = now
            self.get_logger().warn(
                f'🎯 TERMINAL PHASE ENGAGED | dist={self.gps_err_y:.1f}*50m, '
                f'lock_time={(now - self.lock_start_time).nanoseconds/1e9:.1f}s'
            )

    def exit_terminal_phase(self, reason="unknown"):
        """Terminal faze'den çık (Faze–2 → Faze–1)"""
        now = self.get_clock().now()
        if self.flight_phase == 2:
            terminal_duration = (now - self.terminal_engage_time).nanoseconds / 1e9
            self.flight_phase = 1
            self.phase_enter_time = now
            self.terminal_condition_met = False
            self.get_logger().info(
                f'↩️  EXITING TERMINAL PHASE | reason={reason}, duration={terminal_duration:.1f}s'
            )

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.02  # 50 Hz nominal
        self.last_time = now

        # ✅ DEBUG: İlk cycle'da koordinatları kontrol et
        if self.setpoint_count == 0:
            print(f"DEBUG: Own Lat: {self.own_lat}, Leader Lat: {self.leader_lat}")
            print(f"DEBUG: Own Lon: {self.own_lon}, Leader Lon: {self.leader_lon}")
            print(f"DEBUG: Own Alt: {self.own_alt}, Leader Alt: {self.leader_alt}")

        # --- TIME BASE (ALWAYS DEFINED) ---
        time_diff = (now - self.last_target_time).nanoseconds / 1e9

        # --- GPS UPDATE (GPS Mode aktifse) ---
        if self.gps_track_mode:
            self.update_gps_target()

        # --- TARGET VALIDATION ---
        # GPS Mode veya Vision Mode seç
        if self.gps_track_mode:
            # GPS modundayız
            target_valid = self.gps_target_valid
            if target_valid:
                t = {'err_x': self.gps_err_x, 'err_y': self.gps_err_y}  # Simulate vision box
        else:
            # Vision modunda (eski mantık)
            target_valid = (time_diff < 1.0) and (self.last_target is not None)
            if target_valid:
                t = self.last_target

        # Base throttle for fixed-wing cruise (used in target computations)
        base_throttle = 0.6

        # DEBUG: Log offboard state periodically
        if self.setpoint_count % 50 == 0:  # Every ~1 second at 100 Hz
            mode_str = "GPS" if self.gps_track_mode else "VISION"
            target_str = "✓" if target_valid else "✗"
            self.get_logger().info(
                f'🔍 STATE: armed={self.is_armed}, offboard={self.is_offboard}, '
                f'target={target_str}, roll_cmd={math.degrees(self.last_roll_cmd):.1f}° | {mode_str} mode | '
                f'own=({self.own_lat:.4f},{self.own_lon:.4f}) leader=({self.leader_lat},{self.leader_lon})'
            )

        # --- CONTROL LOGIC (target generation) ---
        if target_valid:
            # GPS veya Vision'dan gelen box
            if self.gps_track_mode:
                # GPS eri üretilmiş
                err_x = self.gps_err_x
                err_y = self.gps_err_y
            else:
                # Vision box'ından
                err_x = (t['cx'] - 0.5) * self.fov_rad_x
                err_y = (0.5 - t['cy']) * (self.fov_rad_x * 0.75)
            
            # Distance error (GPS veya Vision)
            if self.gps_track_mode:
                err_dist = 0.0  # GPS modunda distance error zaten err_y'de
            else:
                err_dist = self.target_dist - t['dist']

            # 2. PID
            target_roll = self.roll_pid.update(err_x, dt)
            target_pitch = self.pitch_pid.update(err_y, dt)
            # --- WEEK 3: 4s Target Lock Logic ---
            lock_elapsed = 0.0
            if self.lock_start_time is not None:
              lock_elapsed = (now - self.lock_start_time).nanoseconds / 1e9
            if lock_elapsed >= self.lock_time_required:
              self.target_locked = True
              self.get_logger().info('🎯 ⚡ TARGET LOCKED ⚡ 🎯')
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
            # --- Hedef Kaybı: LOITER DAIRE (20° roll) ---
            # --- WEEK 3: hedef kaybı ek ---
            self.lock_start_time = None
            self.target_locked = False

            # ✅ TARGET KAYBINDA: Sabit 20° roll ile daire çiz (search pattern)
            target_roll = math.radians(20.0)  # Saat yönü daire
            target_pitch = math.radians(2.0)  # Hafif tırmanış
            target_throttle = 0.65  # Cruise throttle
        

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
        
        # --- FW OFFBOARD FIX #2: MIN BANK (ONLY WITH TARGET) ---
        # ✅ FIX: MIN BANK SADECE TARGET VARSA UYGULANIR
        # Target yokken roll = 0 olmalı (daire çizmeyi önler)
        if self.is_offboard and target_valid:
            # Target varsa → bank zorunlu (PX4 failsafe önle)
            if abs(target_roll) < math.radians(3.0):
                target_roll = math.copysign(math.radians(3.0), target_roll if target_roll != 0 else 1.0)
        elif self.is_offboard and not target_valid:
            # Target yoksa → bank zorlama YOK, roll = 0 olabilir
            pass  # target_roll zaten 0'a ayarlı
        
        # --- FW OFFBOARD FIX #1: YAW CONTROL KALDIRILDI ---
        # PX4 FW offboard attitude control'da yaw alanını boş bırak
        # Coordinated turn zaten roll ile yaw'ı çözer
        # (self.target_yaw_integrated ve yaw_sp_move_rate artık kullanılmıyor)

        q_des = self.euler_to_quaternion(target_roll, target_pitch, 0.0)

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
        
        # ✅ TEŞHIS: Throttle'ı teşhis için sabitlenmiş değer (0.7)
        # OFFBOARD düşerse bunu değiştir:
        # safe_throttle = 0.7  # Sabit değer
        # Şu an: dynamic throttle
        safe_throttle = max(target_throttle, 0.1)
        att_msg.thrust_body = [float(safe_throttle), 0.0, 0.0]
        # NOTE: yaw_sp_move_rate kaldırıldı (FW offboard için gerekli değil)
        
        # --- DEBUG: ROLL_CMD KONTROL (Daire çizme teşhisi) ---
        if self.setpoint_count % 100 == 0:
            roll_deg = math.degrees(target_roll)
            if abs(roll_deg) > 0.5:
                self.get_logger().warn(f'⚠️  ROLL_CMD = {roll_deg:.2f}° (target_valid={target_valid})')
            else:
                self.get_logger().info(f'✈️  ROLL_CMD = {roll_deg:.2f}° (düz uçuş)')
        
        # ============================================
        # ✅ FAZE-BAZLI SETPOINT PUBLISH (PX4 FW)
        # ============================================
        # KRİTİK KURAL: Aynı anda iki setpoint YOLLANMAZ!
        #
        # Faze–1: trajectory_setpoint SADECE
        # Faze–2: attitude_setpoint SADECE
        # ============================================
        
        # --- Terminal faze koşulları kontrol et ---
        # GPS'ten mesafe hesapla
        actual_dist = abs(self.gps_err_y) * 50.0 if self.gps_target_valid else float('inf')
        
        # Terminal koşulları sağlanırsa Faze–2'ye gir
        if self.flight_phase == 1 and self.check_terminal_conditions(actual_dist):
            self.enter_terminal_phase()
        
        # Terminal fazında kalma süresi aşıldıysa veya target kaybedildiyse çık
        if self.flight_phase == 2:
            time_in_terminal = (now - self.terminal_engage_time).nanoseconds / 1e9
            if time_in_terminal > self.terminal_duration_max:
                self.exit_terminal_phase("max_time_exceeded")
            elif not target_valid:
                self.exit_terminal_phase("target_lost")
        
        # --- SETPOINT PUBLISH (Faza göre) ---
        if self.flight_phase == 1:
            # ✅ FAZ–1: Trajectory (seyir/takip)
            # SADECE trajectory_setpoint publish et
            traj_msg = TrajectorySetpoint()
            traj_msg.timestamp = 0
            
            # ✅ FIX: Lider bearing'ine göre velocity vector hesapla
            # Eğer GPS target geçerli ise lider bearing'i kullan, yoksa kendi heading'i
            if self.gps_target_valid and self.leader_lat is not None:
                # Leader'a doğru heading
                _, leader_bearing = self.bearing_and_distance(
                    self.own_lat, self.own_lon,
                    self.leader_lat, self.leader_lon
                )
                vx = math.cos(leader_bearing) * self.cruise_speed
                vy = math.sin(leader_bearing) * self.cruise_speed
                target_yaw = leader_bearing
            else:
                # Başlang. heading (kendi heading)
                vx = math.cos(self.own_heading) * self.cruise_speed
                vy = math.sin(self.own_heading) * self.cruise_speed
                target_yaw = self.own_heading
            
            traj_msg.velocity = [float(vx), float(vy), 0.0]
            # ✅ Yaw: Lider bearing'i ata (NaN yerine gerçek açı)
            traj_msg.yaw = float(target_yaw)
            traj_msg.yawspeed = 0.0
            # ✅ FIX: Position: NaN yerine 0,0,0 (velocity-based control için)
            traj_msg.position = [0.0, 0.0, 0.0]
            self.traj_sp_pub.publish(traj_msg)
            
            if self.setpoint_count % 100 == 0:
                self.get_logger().info(
                    f'📍 FAZ–1 (TRAJECTORY): vel=[{vx:.1f}, {vy:.1f}, 0] m/s, yaw={math.degrees(target_yaw):.1f}°, '
                    f'terminal_soon={self.terminal_condition_met}, leader_valid={self.gps_target_valid}'
                )
        
        elif self.flight_phase == 2:
            # ✅ FAZ–2: Attitude (terminal/kamikaze/hassas manevra)
            # SADECE attitude_setpoint publish et (trajectory KESMEK zorunlu!)
            # Pitch agresif yapılabilir (-30° ~ -45°)
            # Thrust: sabit high
            
            terminal_pitch = math.radians(-35.0)  # Agresif dive
            terminal_thrust = 0.85  # Yüksek throttle
            
            q_terminal = self.euler_to_quaternion(0.0, terminal_pitch, 0.0)
            att_terminal = VehicleAttitudeSetpoint()
            att_terminal.timestamp = 0
            att_terminal.q_d = [float(q) for q in q_terminal]
            att_terminal.thrust_body = [float(terminal_thrust), 0.0, 0.0]
            self.attitude_sp_pub.publish(att_terminal)
            
            if self.setpoint_count % 50 == 0:
                self.get_logger().warn(
                    f'🎯 FAZ–2 (ATTITUDE): pitch=-35°, thrust=0.85, '
                    f'time_in_phase={(now - self.terminal_engage_time).nanoseconds/1e9:.1f}s'
                )
            
            if self.setpoint_count % 50 == 0:
                self.get_logger().warn(
                    f'🎯 FAZ–2 (ATTITUDE): pitch=-35°, thrust=0.85, '
                    f'time_in_phase={(now - self.terminal_engage_time).nanoseconds/1e9:.1f}s'
                )
        
        # --- OffboardControlMode (her fazda aynı) ---
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = 0
        
        if self.flight_phase == 1:
            # Faz–1: Velocity mode
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
        elif self.flight_phase == 2:
            # Faz–2: Attitude mode
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = True
        
        offboard_msg.body_rate = False
        offboard_msg.thrust_and_torque = False
        offboard_msg.direct_actuator = False
        self.offboard_mode_pub.publish(offboard_msg)
        
        self.setpoint_count += 1

        # --- FW OFFBOARD FIX #3: ONE-SHOT COMMAND (no spam) ---
        # Setpoint yeterli sayıda gönderildikten sonra ARM/OFFBOARD ONE-SHOT
        if self.setpoint_count >= 20:  # İlk 20 cycle setpoint stream
            # ARM komutu (only once)
            if not self.is_armed and not self.arm_requested:
                self.get_logger().info('🔓 Arming...')
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.arm_requested = True
            
            # OFFBOARD komutu (only once, after armed)
            if self.is_armed and not self.is_offboard and not self.offboard_requested:
                self.get_logger().info('🛸 Requesting OFFBOARD...')
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.offboard_requested = True
            
            # Status log
            if self.setpoint_count % 50 == 0:
                self.get_logger().info(
                    f'✈️  Flight: armed={self.is_armed}, offboard={self.is_offboard}, '
                    f'phase={self.flight_phase}, roll_cmd={math.degrees(self.last_roll_cmd):.1f}°'
                )

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
