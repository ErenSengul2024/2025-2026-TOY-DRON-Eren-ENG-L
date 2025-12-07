import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import ActuatorControl, State
from mavros_msgs.srv import CommandBool, SetMode
import time
import math

class PID:
    def _init_(self, kp=0.0, ki=0.0, kd=0.0, imax=None, out_min=None, out_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_err = None
        self.imax = imax
        self.out_min = out_min
        self.out_max = out_max

    def reset(self):
        self.integral = 0.0
        self.last_err = None

    def update(self, err, dt):
        if dt <= 0.0:
            return 0.0
        p = self.kp * err
        self.integral += err * dt
        if self.imax is not None:
            self.integral = max(min(self.integral, self.imax), -self.imax)
        i = self.ki * self.integral
        d = 0.0
        if self.last_err is not None:
            d = self.kd * (err - self.last_err) / dt
        self.last_err = err
        out = p + i + d
        if self.out_min is not None:
            out = max(out, self.out_min)
        if self.out_max is not None:
            out = min(out, self.out_max)
        return out

class FixedWingPID(Node):
    def _init_(self):
        super()._init_('fixed_wing_pid_controller')

        self.declare_parameter('xYel', 0.5)
        self.declare_parameter('yYel', 0.8)
        self.declare_parameter('xRed', 0.055)
        self.declare_parameter('yRed', 0.055)

        self.declare_parameter('roll_kp', 2.0)
        self.declare_parameter('roll_ki', 0.0)
        self.declare_parameter('roll_kd', 0.5)

        self.declare_parameter('pitch_kp', 1.5)
        self.declare_parameter('pitch_ki', 0.0)
        self.declare_parameter('pitch_kd', 0.2)

        self.declare_parameter('throttle_kp', 0.6)
        self.declare_parameter('throttle_ki', 0.0)
        self.declare_parameter('throttle_kd', 0.05)

        self.xYel = self.get_parameter('xYel').value
        self.yYel = self.get_parameter('yYel').value
        self.xRed = self.get_parameter('xRed').value
        self.yRed = self.get_parameter('yRed').value

        self.roll_pid = PID(
            kp=self.get_parameter('roll_kp').value,
            ki=self.get_parameter('roll_ki').value,
            kd=self.get_parameter('roll_kd').value,
            imax=0.2, out_min=-1.0, out_max=1.0
        )
        self.pitch_pid = PID(
            kp=self.get_parameter('pitch_kp').value,
            ki=self.get_parameter('pitch_ki').value,
            kd=self.get_parameter('pitch_kd').value,
            imax=0.2, out_min=-0.5, out_max=0.5
        )
        self.throttle_pid = PID(
            kp=self.get_parameter('throttle_kp').value,
            ki=self.get_parameter('throttle_ki').value,
            kd=self.get_parameter('throttle_kd').value,
            imax=0.2, out_min=0.0, out_max=1.0
        )

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.actuator_pub = self.create_publisher(ActuatorControl, '/mavros/actuator_control', 10)

        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_setmode = self.create_client(SetMode, '/mavros/set_mode')

        self.target_sub = self.create_subscription(Float32MultiArray, '/target/box', self.target_cb, 10)

        self.last_time = self.get_clock().now()
        self.current_state = None
        self.last_target = None

        self.publish_rate = 50.0  # Hz
        self.timer = self.create_timer(1.0/self.publish_rate, self.control_loop)

        self.get_logger().info('FixedWingPID node started')

    def state_cb(self, msg: State):
        self.current_state = msg

    def target_cb(self, msg: Float32MultiArray):
        arr = msg.data
        if len(arr) < 5:
            self.get_logger().warning('target/box msg length <5; ignoring')
            return
        self.last_target = {
            'cx': float(arr[0]), 'cy': float(arr[1]),
            'w': float(arr[2]), 'h': float(arr[3]),
            'distance': float(arr[4])
        }

    def ensure_offboard_and_arm(self):
        if self.current_state is None:
            return False
        if self.current_state.mode != 'OFFBOARD':
            if self.cli_setmode.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.cli_setmode.call_async(req)

        if not self.current_state.armed:
            if self.cli_arm.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self.cli_arm.call_async(req)
        return True

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1.0/self.publish_rate
        self.last_time = now

        self.ensure_offboard_and_arm()

        if self.last_target is None:
            self.publish_neutral()
            return

        t = self.last_target
        cx, cy, w, h, dist = t['cx'], t['cy'], t['w'], t['h'], t['distance']

        red_ok_x = (w >= self.xRed)
        red_ok_y = (h >= self.yRed)
        red_ok = red_ok_x and red_ok_y

        err_x = (0.5 - cx)
        err_y = (0.5 - cy)

        roll_cmd = self.roll_pid.update(err_x, dt)
        pitch_cmd = self.pitch_pid.update(err_y, dt)
        throttle_cmd = self.throttle_pid.update(0.0, dt)  # örnek: size_err yoksa 0

        aileron = float(max(min(roll_cmd, 1.0), -1.0))
        elevator = float(max(min(pitch_cmd, 0.5), -0.5))
        rudder = 0.0
        base_throttle = 0.45
        throttle = float(max(min(base_throttle + throttle_cmd, 1.0), 0.0))

        inside_x_yellow = (abs(cx - 0.5) <= (self.xYel/2.0))
        inside_y_yellow = (abs(cy - 0.5) <= (self.yYel/2.0))
        if red_ok and inside_x_yellow and inside_y_yellow:
            aileron *= 0.3
            elevator *= 0.3
            throttle = base_throttle

        act = ActuatorControl()
        act.group_mix = 0
        act.controls = [aileron, elevator, rudder, throttle, 0.0, 0.0, 0.0, 0.0]

        self.actuator_pub.publish(act)

    def publish_neutral(self):
        act = ActuatorControl()
        act.group_mix = 0
        act.controls = [0.0, 0.0, 0.0, 0.45, 0.0, 0.0, 0.0, 0.0]
        self.actuator_pub.publish(act)


def main(args=None):
    rclpy.init(args=args)
    node = FixedWingPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down FixedWingPID')
    node.destroy_node()
    rclpy.shutdown()


if _name_ == '_main_':
    main()
