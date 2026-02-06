,import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleAttitudeSetpoint, OffboardControlMode, VehicleStatus, VehicleCommand, VehicleAttitude, VehicleLocalPosition, VehicleAirData, VehicleGlobalPosition
from sensor_msgs.msg import NavSatFix
import time
import math
import numpy as np

class FixedWingFinalDiamond(Node):
    def __init__(self):
        super().__init__('fixed_wing_final_diamond')

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
        self.status_sub = self.create_subscription(VehicleStatus, '/px4_0/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)
        # ATTITUDE SUBSCRIBER DISABLED: /fmu/out/vehicle_attitude_v1 not available - will use integrated yaw only
        # self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude_v1', self.attitude_cb, qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/px4_0/fmu/out/vehicle_local_position_v1', self.local_pos_cb, qos_profile)
        self.air_data_sub = self.create_subscription(VehicleAirData, '/px4_0/fmu/out/vehicle_air_data_v1', self.air_data_cb, qos_profile)
        # ⭐ KRİTİK FİKS: Kendi GPS konumunu dinle
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, '/px4_0/fmu/out/vehicle_global_position', self.global_pos_cb, qos_profile)
        self.target_sub = self.create_subscription(Float32MultiArray, '/target/box', self.target_cb, 10)
        
        # İKİNCİ UÇAK GPS SUBSCRIBER: İkinci uçağın GPS verilerini dinle
        # px4_1 namespace'indeki ikinci uçağın GPS'i
        self.second_drone_gps_sub = self.create_subscription(VehicleGlobalPosition, '/px4_1/fmu/out/vehicle_global_position', self.second_drone_gps_cb, qos_profile)
        
        self.attitude_sp_pub = self.create_publisher(VehicleAttitudeSetpoint, '/px4_0/fmu/in/vehicle_attitude_setpoint_v1', qos_profile)
        # NOTE: PX4 expects OffboardControlMode on '/fmu/in/offboard_control_mode' (no _v1)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/px4_0/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/px4_0/fmu/in/vehicle_command_v1', qos_profile)

        # --- State Variables ---
        self.last_time = self.get_clock().now()
        self.last_target_time = self.get_clock().now()
        self.last_target = None
        
        # İKİNCİ UÇAK STATE VARIABLES
        self.second_drone_gps = None  # NavSatFix mesajı
        self.second_drone_local_pos = None  # İkinci uçağın lokal pozisyonu (hesaplanacak)
        self.first_drone_gps = None  # Birinci uçağın GPS'i (referans)
        self.use_second_drone_as_target = False  # Mode seçimi: True ise target olarak ikinci uçak
        self.last_second_drone_update = self.get_clock().now()
        
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

    def global_pos_cb(self, msg: VehicleGlobalPosition):
        """
        ⭐ KRİTİK FİKS: Kendi uçağının GPS konumunu güncelle
        Bu callback sayesinde first_drone_gps değişkeni artık None kalmayacak
        ve gps_to_local_position fonksiyonu doğru referans noktası ile çalışacak.
        
        VehicleGlobalPosition mesajında lat, lon, alt bilgisi bulunur.
        """
        # VehicleGlobalPosition'ın attribute'ları: lat, lon, alt (int32 ölçekli veya float)
        # px4_msgs kaynak koduna göre, genellikle derece cinsinden ve 1e-7 ölçeklidir
        # Ancak, node tarafından ROS float'a çevrilir, bu yüzden derece olarak alırız
        if hasattr(msg, 'lat') and hasattr(msg, 'lon'):
            # px4_msgs raw format: lat/lon int32 (1e-7 ölçekli)
            lat_deg = msg.lat / 1e7 if msg.lat > 1000 else msg.lat
            lon_deg = msg.lon / 1e7 if msg.lon > 1000 else msg.lon
            alt_m = msg.alt / 1000.0 if msg.alt > 1000 else msg.alt
        else:
            # Fallback: latitude/longitude attributes
            lat_deg = getattr(msg, 'latitude', 0.0)
            lon_deg = getattr(msg, 'longitude', 0.0)
            alt_m = getattr(msg, 'altitude', 0.0)
        
        # NavSatFix benzeri bir object oluştur referans olarak
        # (Böylece gps_to_local_position fonksiyonu uyumlu kalır)
        self.first_drone_gps = type('GPS', (), {
            'latitude': lat_deg,
            'longitude': lon_deg,
            'altitude': alt_m
        })()
        
        if self.setpoint_count % 100 == 0:
            self.get_logger().debug(f'Own GPS: Lat={lat_deg:.6f}, Lon={lon_deg:.6f}, Alt={alt_m:.1f}m')

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
        self.ground_speed = math.sqrt(msg.vx**2 + msg.vy**2)
        self.current_alt = -msg.z # NED coordinate system (z is negative up)
        
        # İlk kez cihaz başladığında, GPS referansı olarak kendi konumunu kaydet
        # (Bu gerçek uygulamada NavSatFix'ten alınmalı)
        if self.first_drone_gps is None and self.second_drone_gps is not None:
            # Dummy: Gerçekte birinci uçağın GPS'i subscribe edilmeli
            # Şimdilik ikinci uçak GPS'i alındığında referans olarak kullan
            pass
    
    def global_pos_cb(self, msg: VehicleGlobalPosition):
        """
        Kendi uçağın GPS koordinatlarını al. Bu, ikinci uçak takibi için referans noktası olacak.
        VehicleGlobalPosition: lat/lon (1e7 scaled), alt (mm cinsinden)
        """
        self.first_drone_gps = msg
        if self.setpoint_count % 100 == 0:
            lat_deg = msg.lat / 1e7
            lon_deg = msg.lon / 1e7
            alt_m = msg.alt / 1000.0
            self.get_logger().debug(f'Own GPS: Lat={lat_deg:.6f}, Lon={lon_deg:.6f}, Alt={alt_m:.2f}m')

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

    def second_drone_gps_cb(self, msg: VehicleGlobalPosition):
        """
        İkinci uçağın GPS verilerini al.
        VehicleGlobalPosition: lat/lon (1e7 scaled), alt (mm cinsinden)
        """
        self.second_drone_gps = msg
        self.last_second_drone_update = self.get_clock().now()
        
        if self.setpoint_count % 100 == 0 and self.second_drone_gps is not None:
            lat_deg = msg.lat / 1e7
            lon_deg = msg.lon / 1e7
            alt_m = msg.alt / 1000.0
            self.get_logger().debug(f'2nd Drone GPS: Lat={lat_deg:.6f}, Lon={lon_deg:.6f}, Alt={alt_m:.2f}m')
        
    def gps_to_local_position(self, gps_msg: NavSatFix, reference_gps=None):
        """
        GPS koordinatlarını lokal NED frame'e çevir.
        reference_gps: Referans noktası (None ise orijin (0,0,0) kabul edilir)
        
        ⭐ UYARANDI: reference_gps None ise devasa sayılar üretir (1e7 bug)
        Lütfen reference_gps'i güncellenmiş first_drone_gps ile kullan!
        
        Returns: (north, east, down) - metre cinsinden lokal pozisyon
        """
        if gps_msg is None:
            return 0.0, 0.0, 0.0
            
        if reference_gps is None:
            # ⚠️ UYARISI: Bu durum HATAYA yol açar! Ref GPS'i None göndermeme!
            self.get_logger().warn('GPS reference is None! Using (0,0,0) as origin - this may be wrong!')
            # Güvenli fallback
            north = 0.0
            east = 0.0
            down = -gps_msg.altitude if hasattr(gps_msg, 'altitude') else 0.0
        else:
            # Attribute uyumluluğu kontrol et (NavSatFix ve VehicleGlobalPosition için)
            try:
                lat_target = gps_msg.latitude if hasattr(gps_msg, 'latitude') else gps_msg.lat / 1e7
                lon_target = gps_msg.longitude if hasattr(gps_msg, 'longitude') else gps_msg.lon / 1e7
                alt_target = gps_msg.altitude if hasattr(gps_msg, 'altitude') else gps_msg.alt / 1000.0
            except Exception as e:
                self.get_logger().error(f'Error accessing target GPS: {e}')
                return 0.0, 0.0, 0.0
            
            try:
                lat_ref = reference_gps.latitude if hasattr(reference_gps, 'latitude') else reference_gps.lat / 1e7
                lon_ref = reference_gps.longitude if hasattr(reference_gps, 'longitude') else reference_gps.lon / 1e7
                alt_ref = reference_gps.altitude if hasattr(reference_gps, 'altitude') else reference_gps.alt / 1000.0
            except Exception as e:
                self.get_logger().error(f'Error accessing reference GPS: {e}')
                return 0.0, 0.0, 0.0
            
            # Haversine/basit euclidean uzaklık hesabı
            lat_to_m = 111132.92  # metres per degree latitude
            lon_to_m = 111412.84 * math.cos(math.radians((lat_target + lat_ref) / 2.0))
            
            north = (lat_target - lat_ref) * lat_to_m
            east = (lon_target - lon_ref) * lon_to_m
            down = -(alt_target - alt_ref)
        
        return north, east, down
    
    def local_position_to_target(self, local_pos, vehicle_pos, vehicle_yaw):
        """
        Lokal pozisyonu, uçağın pozisyonu ve yaw bilgisi kullanarak
        kameraya göre target box'a çevir.
        
        Parameters:
            local_pos: (north, east, down) - hedefin lokal pozisyonu
            vehicle_pos: (north, east, down) - uçağın lokal pozisyonu  
            vehicle_yaw: radyan cinsinden yaw açısı
        
        Returns: {'cx': normalized_x, 'cy': normalized_y, 'dist': distance_m}
                 cx, cy: 0-1 arasında normalize edilmiş kamera koordinatları
                 dist: metre cinsinden uzaklık
        """
        # Hedefin uçağa göre göreceli pozisyonu
        rel_north = local_pos[0] - vehicle_pos[0]
        rel_east = local_pos[1] - vehicle_pos[1]
        rel_down = local_pos[2] - vehicle_pos[2]
        
        # Uçağın body frame'ine döndür (yaw ile)
        cos_yaw = math.cos(vehicle_yaw)
        sin_yaw = math.sin(vehicle_yaw)
        
        # Body frame: forward (x), right (y), down (z)
        body_forward = cos_yaw * rel_north + sin_yaw * rel_east
        body_right = -sin_yaw * rel_north + cos_yaw * rel_east
        body_down = rel_down
        
        # Body frame'de uzaklık (forward, right, down)
        distance_m = math.sqrt(body_forward**2 + body_right**2 + body_down**2)
        
        # Kamera FOV'u kullanarak kamera koordinatlarına çevir
        if distance_m > 0.1:  # Çok yakın değilse
            # Forward'a göre açılar
            angle_down = math.atan2(body_down, body_forward)  # Aşağı açısı
            angle_right = math.atan2(body_right, body_forward)  # Sağ açısı
            
            # Normalize et (FOV'a göre)
            # FOV'un ortası 0.5, kenarları 0-1 arasında
            cx = 0.5 + angle_right / self.fov_rad_x
            cy = 0.5 - angle_down / (self.fov_rad_x * 0.75)  # y ters (kamera koordinatı)
            
            # Clamp to valid range
            cx = max(0.0, min(1.0, cx))
            cy = max(0.0, min(1.0, cy))
        else:
            cx, cy = 0.5, 0.5  # Kamera merkezinde
        
        return {'cx': cx, 'cy': cy, 'dist': distance_m}

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
    
    def set_target_mode(self, mode: str):
        """
        Target modunu değiştir.
        mode: 'external_box' veya 'second_drone'
        """
        if mode == 'second_drone':
            self.use_second_drone_as_target = True
            self.get_logger().info('MODE CHANGED: Tracking 2nd Drone GPS')
        elif mode == 'external_box':
            self.use_second_drone_as_target = False
            self.get_logger().info('MODE CHANGED: Tracking External Target Box')
        else:
            self.get_logger().warn(f'Unknown mode: {mode}')

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
        # Birinci target: External target box
        time_diff = (now - self.last_target_time).nanoseconds / 1e9
        target_valid = (time_diff < 1.0) and (self.last_target is not None)
        
        # İkinci target: İkinci uçak GPS'den hesaplanan target
        time_diff_second = (now - self.last_second_drone_update).nanoseconds / 1e9
        second_drone_valid = (time_diff_second < 2.0) and (self.second_drone_gps is not None) and (self.current_alt > 0.1)
        
        # Target seçme: Hangi hedefi takip edeceğiz?
        # Eğer ikinci uçak target mode aktifse ve GPS verisi varsa, onu kullan
        # Aksi takdirde external target box'ı kullan
        current_target = None
        
        if self.use_second_drone_as_target and second_drone_valid:
            # İkinci uçağı hedef olarak kullan
            second_drone_local = self.gps_to_local_position(self.second_drone_gps, self.first_drone_gps)
            vehicle_local = (0, 0, -self.current_alt)  # Mevcut uçak lokal frame orijininde
            
            # ⭐ KRİTİK: first_drone_gps None ise bu hesaplama hatalı sonuç verir
            if self.first_drone_gps is None:
                self.get_logger().warn('First drone GPS not yet available - cannot calculate target from 2nd drone')
                current_target = None
            else:
                current_target = self.local_position_to_target(
                    second_drone_local, 
                    vehicle_local, 
                    self.current_yaw
                )
                if self.setpoint_count % 50 == 0:
                    self.get_logger().info(f'TARGET MODE: 2nd Drone (Dist={current_target["dist"]:.1f}m)')
        elif target_valid:
            # External target box'ı kullan
            current_target = self.last_target
            if self.setpoint_count % 50 == 0:
                self.get_logger().info(f'TARGET MODE: External Box (Dist={current_target["dist"]:.1f}m)')
        else:
            if self.setpoint_count % 50 == 0:
                self.get_logger().warn('NO TARGET AVAILABLE - Loiter mode')

        # Base throttle for fixed-wing cruise (used in target computations)
        base_throttle = 0.6

        # DEBUG: Log offboard state periodically
        if self.setpoint_count % 50 == 0:  # Every ~1 second at 50 Hz
            self.get_logger().info(
                f'STATE: armed={self.is_armed}, offboard={self.is_offboard}, '
                f'setpoint_count={self.setpoint_count}'
            )

        # --- CONTROL LOGIC (target generation) ---
        if current_target is not None:
            t = current_target
            
            # 1. FOV Tabanlı Açı Hesabı
            angle_err_x = (t['cx'] - 0.5) * self.fov_rad_x
            angle_err_y = (0.5 - t['cy']) * (self.fov_rad_x * 0.75)
            err_dist = self.target_dist - t['dist']

            # 2. PID
            target_roll = self.roll_pid.update(angle_err_x, dt)
            target_pitch = self.pitch_pid.update(angle_err_y, dt)
            
            # 3. Energy Mixing (TECS Emulation + Bank Compensation)
            # Pitch Up -> Throttle Up
            pitch_throttle_ff = max(0.0, target_pitch) * (0.25 / math.radians(20))
            # Bank Angle -> Throttle Up (Dönüşte irtifa kaybını önle)
            roll_throttle_ff = abs(target_roll) * (0.15 / math.radians(45))
            
            pid_thr = self.throttle_pid.update(err_dist, dt)
            target_throttle = base_throttle + pid_thr + pitch_throttle_ff + roll_throttle_ff

        else:
            # --- Hedef Kaybı: Smooth Decay & Loiter ---
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
    def __init__(self, kp, ki=0.0, kd=0.0, imax=0.0, out_min=-1.0, out_max=1.0):
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

if __name__ == '__main__':
    main()

