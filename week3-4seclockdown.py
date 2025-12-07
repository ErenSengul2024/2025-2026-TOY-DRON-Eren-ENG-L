import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from collections import deque
import time

class LockTester(Node):
    def __init__(self):
        super().__init__('lock_tester')

        self.xYel = 0.5
        self.yYel = 0.8


        self.x_min = 0.5 - self.xYel / 2.0
        self.x_max = 0.5 + self.xYel / 2.0
        self.y_min = 0.5 - self.yYel / 2.0
        self.y_max = 0.5 + self.yYel / 2.0


        self.required_locked_time = 4.0
        self.window_size = 5.0         


        self.red_sub = self.create_subscription(
            Float32MultiArray, '/target_bbox', self.red_cb, 10)

        self.lock_event_pub = self.create_publisher(String, '/lock_event', 10)
        self.approved_pub = self.create_publisher(Bool, '/lock_approved', 10)

        self.locked = False
        self.lock_start_time = None
        self.lock_history = deque()
        self.approved_published = False

        self.get_logger().info("Lock Tester started with fixed yellow box.")


    def red_cb(self, msg):
        data = msg.data
        if len(data) < 2:
            return

        cx, cy = float(data[0]), float(data[1])
        now = time.time()

        inside = self.is_inside_yellow(cx, cy)


        if inside and not self.locked:
            self.locked = True
            self.lock_start_time = now
            self.publish_event(f"START {now:.6f}")


        elif (not inside) and self.locked:
            start = self.lock_start_time
            end = now
            self.lock_history.append((start, end))
            self.locked = False
            self.lock_start_time = None
            self.publish_event(f"END {end:.6f}")


        locked_time = self.compute_locked(now)

        if (not self.approved_published) and locked_time >= self.required_locked_time:
            self.publish_approval(now, locked_time)


        self.prune_history(now)

    def is_inside_yellow(self, cx, cy):
        return (
            self.x_min <= cx <= self.x_max and
            self.y_min <= cy <= self.y_max
        )

    def compute_locked(self, now):
        window_start = now - self.window_size
        total = 0.0


        for (s, e) in self.lock_history:
            if e <= window_start:
                continue
            seg_start = max(s, window_start)
            seg_end = e
            if seg_end > seg_start:
                total += seg_end - seg_start


        if self.locked and self.lock_start_time is not None:
            seg_start = max(self.lock_start_time, window_start)
            seg_end = now
            if seg_end > seg_start:
                total += seg_end - seg_start

        return total


    def prune_history(self, now):
        window_start = now - self.window_size
        while self.lock_history and self.lock_history[0][1] <= window_start:
            self.lock_history.popleft()


    def publish_event(self, text):
        msg = String()
        msg.data = text
        self.lock_event_pub.publish(msg)
        self.get_logger().info(text)


    def publish_approval(self, now, total):
        self.approved_published = True
        msg = Bool()
        msg.data = True
        self.approved_pub.publish(msg)
        self.get_logger().info(
            f"LOCK APPROVED at {now:.3f} (locked={total:.3f}s)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LockTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
