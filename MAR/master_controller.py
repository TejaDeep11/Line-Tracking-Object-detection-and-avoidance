import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.line_sub = self.create_subscription(Float32, '/line_error', self.line_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.state = 'FOLLOW_LINE'
        self.line_error = 0.0
        self.line_visible = False
        self.line_timeout = 0
        
        self.min_front_dist = 999.0
        self.left_dist = 999.0
        self.right_dist = 999.0
        
        self.evade_timer = 0
        self.turn_mult = 1.0 
        
        self.kp = 0.012 
        self.kd = 0.005
        self.prev_error = 0.0
        self.last_known_error = 0.0 
        
        self.lost_timer = 0
        self.evade_cooldown = 0

    def scan_callback(self, msg):
        total_rays = len(msg.ranges)
        center = total_rays // 2
        sweep = total_rays // 6
        
        left_rays = msg.ranges[center : center + sweep]
        right_rays = msg.ranges[center - sweep : center]

        valid_left = [r for r in left_rays if 0.01 < r < 10.0]
        valid_right = [r for r in right_rays if 0.01 < r < 10.0]

        self.left_dist = min(valid_left) if valid_left else 999.0
        self.right_dist = min(valid_right) if valid_right else 999.0
        self.min_front_dist = min(self.left_dist, self.right_dist)

    def line_callback(self, msg):
        self.line_error = msg.data
        self.line_visible = True
        self.line_timeout = 0
        self.lost_timer = 0 

    def control_loop(self):
        vel_msg = Twist()
        self.line_timeout += 1
        if self.line_timeout > 10:
            self.line_visible = False

        if self.evade_cooldown > 0:
            self.evade_cooldown -= 1

        # --- THE SMART FSM ---
        if self.state == 'FOLLOW_LINE':
            if self.min_front_dist < 0.85 and self.evade_cooldown == 0:
                
                # NEW: THE EMERGENCY BRAKE!
                # If BOTH sides detect the object closer than 0.9m, it means the object is dead center.
                if self.left_dist < 0.9 and self.right_dist < 0.9:
                    self.get_logger().error('Obstacle blocking the middle! EMERGENCY STOP.')
                    self.state = 'EMERGENCY_STOP'
                
                # Otherwise, it's just on an edge, so we dodge it normally.
                elif self.right_dist < self.left_dist:
                    self.get_logger().warn('Obstacle on Right. Dodging LEFT.')
                    self.turn_mult = 1.0  
                    self.state = 'SPIN_OUT'
                    self.evade_timer = 0
                else:
                    self.get_logger().warn('Obstacle on Left. Dodging RIGHT.')
                    self.turn_mult = -1.0 
                    self.state = 'SPIN_OUT'
                    self.evade_timer = 0
        
        # NEW: The resume logic!
        elif self.state == 'EMERGENCY_STOP':
            # If the professor removes the obstacle, the robot will automatically resume!
            if self.min_front_dist > 1.0:
                self.get_logger().info('Path cleared! Resuming track.')
                self.state = 'FOLLOW_LINE'
        
        elif self.state == 'SPIN_OUT':
            self.evade_timer += 1
            if self.evade_timer > 25: 
                self.state = 'DRIVE_LATERAL'
                self.evade_timer = 0
                
        elif self.state == 'DRIVE_LATERAL':
            self.evade_timer += 1
            if self.evade_timer > 65: 
                self.state = 'SPIN_IN'
                self.evade_timer = 0
                
        elif self.state == 'SPIN_IN':
            self.evade_timer += 1
            if self.evade_timer > 25: 
                self.state = 'DRIVE_PAST'
                self.evade_timer = 0
                
        elif self.state == 'DRIVE_PAST':
            self.evade_timer += 1
            if self.evade_timer > 160: 
                self.get_logger().info('Bypass complete. Searching for track.')
                self.state = 'FIND_LINE'
                self.evade_timer = 0
                
        elif self.state == 'FIND_LINE':
            self.evade_timer += 1
            if self.evade_timer > 20 and self.line_visible and abs(self.line_error) < 150:
                self.get_logger().info('Track locked! Resuming PID.')
                self.state = 'FOLLOW_LINE'
                self.evade_cooldown = 60 

        # --- THE ACTIONS ---
        if self.state == 'FOLLOW_LINE':
            if self.line_visible:
                steering = (self.kp * self.line_error) + (self.kd * (self.line_error - self.prev_error))
                self.prev_error = self.line_error
                self.last_known_error = self.line_error 
                
                vel_msg.linear.x = 0.1 
                vel_msg.angular.z = -float(steering)
            else:
                vel_msg.linear.x = 0.03 
                self.lost_timer += 1
                
                spin_dir = -0.5 if self.last_known_error > 0 else 0.5
                
                if self.lost_timer < 30: 
                    vel_msg.angular.z = spin_dir
                elif self.lost_timer < 90: 
                    vel_msg.angular.z = -spin_dir
                elif self.lost_timer < 150: 
                    vel_msg.angular.z = spin_dir
                else:
                    self.lost_timer = 0 
        
        # NEW: Slam the brakes!
        elif self.state == 'EMERGENCY_STOP':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            
        elif self.state == 'SPIN_OUT':
            vel_msg.linear.x = 0.0  
            vel_msg.angular.z = 1.0 * self.turn_mult  
            
        elif self.state == 'DRIVE_LATERAL':
            vel_msg.linear.x = 0.2  
            vel_msg.angular.z = 0.0 
            
        elif self.state == 'SPIN_IN':
            vel_msg.linear.x = 0.0  
            vel_msg.angular.z = -1.0 * self.turn_mult 
            
        elif self.state == 'DRIVE_PAST':
            vel_msg.linear.x = 0.2  
            vel_msg.angular.z = 0.0 
            
        elif self.state == 'FIND_LINE':
            vel_msg.linear.x = 0.15
            vel_msg.angular.z = -0.5 * self.turn_mult 

        self.cmd_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
