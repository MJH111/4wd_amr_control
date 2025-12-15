import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from math import atan2, sqrt, pi
import yaml
import os

class CmdVelToMotorKinematics(Node):
    def __init__(self):
        super().__init__('cmd_vel_kinematics_node')
        
        config_file = 'compal_amr.yaml' 
        self._load_config(config_file)

        # Wheel order: FL, FR, RL, RR
        self.wheel_positions = {
            'FL': [self.w_base/2, self.t_width/2],
            'FR': [self.w_base/2, -self.t_width/2],
            'RL': [-self.w_base/2, self.t_width/2],
            'RR': [-self.w_base/2, -self.t_width/2],
        }

        self.cmd = np.zeros(3, dtype=np.float32)
        
        # Subscribes to input commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel', 
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribing to /cmd_vel topic.")

        # Two Publishers for Separate Commands (to bypass 4-element limitation in Isaac Sim)
        self.steering_pub = self.create_publisher(Float32MultiArray, '/amr_steering_angles', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, '/amr_wheel_speeds', 10)
        self.get_logger().info("Publishing steering angles (Angular) to /amr_steering_angles.")
        self.get_logger().info("Publishing wheel speeds (Linear) to /amr_wheel_speeds.")


    def _load_config(self, filename):
        try:
            package_name = 'compal_amr_pkg'
            share_directory = get_package_share_directory(package_name)
            config_path = os.path.join(share_directory, filename)
            
            with open(config_path, "r") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
                
                self.cmd_scale = float(config.get("cmd_scale", 1.0))
                self.w_base = float(config["wheelbase"])
                self.t_width = float(config["trackwidth"])
                
                self.get_logger().info(
                    f"Loaded config: wheelbase={self.w_base:.3f} m, trackwidth={self.t_width:.3f} m, cmd_scale={self.cmd_scale:.1f}"
                )
        except (FileNotFoundError, KeyError) as e:
            self.get_logger().error(f"FATAL: Missing config file or key '{e}'. Cannot proceed.")
            self.cmd_scale = 1.0
            self.w_base = 0.4
            self.t_width = 0.4
            raise SystemExit("Configuration Error: Required parameters not loaded.") from e
        except Exception as e:
            self.get_logger().error(f"Error loading configuration: {e}")
            raise SystemExit("Configuration Error: Failed to parse YAML.") from e


    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        scaled_cmd = np.array([vx, vy, wz], dtype=np.float32) * self.cmd_scale
        
        wheel_ang, wheel_speed = self.compute(scaled_cmd)
        self._publish_motor_command(wheel_ang, wheel_speed)


    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2*np.pi) - np.pi

    def normalize(self, angle, speed):
        angle = self._wrap_angle(angle) 
        if angle > pi/2:
            angle -= pi
            speed *= -1
        elif angle < -pi/2:
            angle += pi
            speed *= -1
        return angle, speed
    
    def compute(self, cmd_vel: np.ndarray):
        vx, vy, wz = cmd_vel
        wheel_angles = []
        wheel_speeds = []
        for name in ['FL', 'FR', 'RL', 'RR']:
            x_offset, y_offset = self.wheel_positions[name]
            delta_vx = -wz * y_offset
            delta_vy = wz * x_offset
            total_vx = vx + delta_vx
            total_vy = vy + delta_vy
            speed = sqrt(total_vx**2 + total_vy**2)
            angle = atan2(total_vy, total_vx)
            angle, speed = self.normalize(angle, speed)

            wheel_speeds.append(speed) # Linear speed (m/s)
            wheel_angles.append(angle) # Steering angle (rad)

        return wheel_angles, wheel_speeds
    
    def _publish_motor_command(self, wheel_ang: list, wheel_speed: list):
        """Publishes 4 angles and 4 speeds on two separate topics."""
        
        # 1. Steering Angles (Angular Command - Position/Angle)
        angle_msg = Float32MultiArray()
        angle_msg.data = wheel_ang
        self.steering_pub.publish(angle_msg)
        
        # 2. Wheel Speeds (Linear Command - Velocity/Speed)
        speed_msg = Float32MultiArray()
        speed_msg.data = wheel_speed
        self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CmdVelToMotorKinematics()
        rclpy.spin(node)
    except SystemExit as e:
        print(f"Node terminated due to error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()