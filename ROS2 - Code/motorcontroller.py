import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Initialize serial communication with Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.flush()

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x  # Forward/backward speed
        angular_z = msg.angular.z  # Rotation speed
        
        # Convert velocity commands to motor speeds (basic differential drive calculation)
        wheel_separation = 0.2  # Distance between wheels (meters)
        left_speed = linear_x - (angular_z * wheel_separation / 2)
        right_speed = linear_x + (angular_z * wheel_separation / 2)
        
        # Scale and convert to integer for Arduino
        left_pwm = int(left_speed * 255)
        right_pwm = int(right_speed * 255)
        
        # Send data to Arduino
        command = f"{left_pwm},{right_pwm}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent: {command}')


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
