import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Listen to the camera's error calculations
        self.subscription = self.create_subscription(
            Float32, '/line_error', self.error_callback, 10)
            
        # Publish velocity commands directly to the robot's wheels
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- PID Constants ---
        # These are the numbers your Control Engineer will need to tune
        self.kp = 0.005  # Proportional: How hard to turn based on current error
        self.ki = 0.000  # Integral: Fixes long-term drift (usually 0 for line followers)
        self.kd = 0.002  # Derivative: Dampens the turning to prevent wobbling
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

    def error_callback(self, msg):
        error = msg.data
        
        # Calculate time difference (dt)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Prevent division by zero on the very first frame
        if dt <= 0.0:
            return 
            
        # 1. Proportional Term
        p_term = self.kp * error
        
        # 2. Integral Term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # 3. Derivative Term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Total steering calculation
        steering = p_term + i_term + d_term
        
        # Save values for the next loop
        self.prev_error = error
        self.last_time = current_time

        # Create the velocity command
        vel_msg = Twist()
        vel_msg.linear.x = 0.15  # Constant forward speed (0.15 meters/second)
        
        # Angular velocity (steering). 
        # Note: If the line is to the right (+ pixel error), we need to turn right (- Z angular).
        vel_msg.angular.z = -float(steering)
        
        # Send the command to the wheels!
        self.publisher.publish(vel_msg)
        
        # Debugging output to your terminal
        self.get_logger().info(f'Error: {error:.2f} | Steering: {vel_msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
