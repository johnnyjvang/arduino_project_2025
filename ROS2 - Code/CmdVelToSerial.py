import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        lin_x = msg.linear.x  # Forward/backward speed
        ang_z = msg.angular.z  # Turning speed

        # Convert to left/right wheel speeds
        wheel_base = 0.16  # Adjust based on your robot
        wheel_radius = 0.03

        left_speed = (lin_x - ang_z * wheel_base / 2) * 60 / (2 * 3.14 * wheel_radius)
        right_speed = (lin_x + ang_z * wheel_base / 2) * 60 / (2 * 3.14 * wheel_radius)

        command = f"m {left_speed},{right_speed}\n"
        self.serial_port.write(command.encode())

def main():
    rclpy.init()
    node = CmdVelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
