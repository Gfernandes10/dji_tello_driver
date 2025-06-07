import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from djitellopy import Tello
import subprocess
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import numpy as np
from dji_tello_msgs.msg import TelloStatus
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class TelloNode(Node):
    def __init__(self):
        super().__init__('tello_node')
        # Connect to the Tello Wi-Fi network
        # Replace 'TELLO-AB12CD' with your Tello's Wi-Fi SSID
        ssid = self.declare_parameter('tello_ssid', 'TELLO-B5ACD2').get_parameter_value().string_value

        self.connect_to_tello_wifi(ssid)

        # Initialize the Tello drone
        self.tello = Tello()
        try:
            self.tello.connect()
            #self.tello.streamon() # Currently this is frozen my computer, I need to understand why
            self.get_logger().info('Conected to Tello!')
        except Exception as e:
            self.get_logger().error(f'Conection to tello was not succesful: {e}')
            return
        
        # Create a publisher for odometry data
        self.odom_pub = self.create_publisher(Odometry, 'tello/odometry', 10)
        self.odom_timer = self.create_timer(0.5, self.publish_odometry)
        # Create a publisher for Tello status
        self.tello_status_pub = self.create_publisher(TelloStatus, 'tello/status', 10)
        self.tello_status_pub_timer = self.create_timer(1.0, self.publish_tello_status)
        # Create subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(Twist, 'tello/cmd_vel', self.cmd_vel_callback, 10)
        # Create a service for takeoff
        self.takeoff_service = self.create_service(Trigger, 'tello/takeoff', self.takeoff_callback)
        # Create a service for landing
        self.land_service = self.create_service(Trigger, 'tello/land', self.land_callback)

        """ # Create a publisher for video stream
        self.video_pub = self.create_publisher(Image, 'tello/image_raw', 10)
        self.video_timer = self.create_timer(1.0/30.0, self.publish_video_stream) # 30 FPS
        # Create a CvBridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()
        self.frame_read = self.tello.get_frame_read() """

    def publish_odometry(self):
        try:
            # Get sensor data
            agx = self.tello.get_acceleration_x()
            agy = self.tello.get_acceleration_y()
            agz = self.tello.get_acceleration_z()
            pitch_degree = self.tello.get_pitch()
            roll_degree = self.tello.get_roll()
            yaw_degree = self.tello.get_yaw()
            vbx = self.tello.get_speed_x()
            vby = self.tello.get_speed_y()
            vbz = self.tello.get_speed_z()

            # Integrate velocity to estimate position
            if not hasattr(self, 'odom_last_time'):
                self.odom_last_time = self.get_clock().now().nanoseconds / 1e9
                self.odom_pos = [0.0, 0.0, 0.0]
            now = self.get_clock().now().nanoseconds / 1e9
            dt = now - self.odom_last_time
            self.odom_last_time = now
            # Integrate only if dt is reasonable
            if 0 < dt < 1.0:
                self.odom_pos[0] += vbx * dt
                self.odom_pos[1] += vby * dt
                self.odom_pos[2] += vbz * dt

            # Create an Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'tello_odom'
            odom_msg.pose.pose.position.x = float(self.odom_pos[0])
            odom_msg.pose.pose.position.y = float(self.odom_pos[1])
            odom_msg.pose.pose.position.z = float(self.odom_pos[2])
            # Orientation (Euler to quaternion)
            q = quaternion_from_euler(
                np.deg2rad(roll_degree),
                np.deg2rad(pitch_degree),
                np.deg2rad(yaw_degree)
            )
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = float(vbx)
            odom_msg.twist.twist.linear.y = float(vby)
            odom_msg.twist.twist.linear.z = float(vbz)

            self.odom_pub.publish(odom_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')
            
    def publish_tello_status(self):
        try:
            msg = TelloStatus()
            msg.battery = self.tello.get_battery()
            #msg.wifi_signal_noise_ratio = self.tello.query_wifi_signal_noise_ratio() 
            msg.flight_time = self.tello.get_flight_time()
            msg.temperature = self.tello.get_temperature()
            msg.highest_temperature = self.tello.get_highest_temperature()
            msg.lowest_temperature = self.tello.get_lowest_temperature()
            msg.agx = self.tello.get_acceleration_x()
            msg.agy = self.tello.get_acceleration_y()
            msg.agz = self.tello.get_acceleration_z()
            msg.barometer_cm = self.tello.get_barometer()
            msg.height = self.tello.get_height()
            msg.speed_x = self.tello.get_speed_x()
            msg.speed_y = self.tello.get_speed_y()
            msg.speed_z = self.tello.get_speed_z()
            self.tello_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error to obtain tello status: {e}')

    def connect_to_tello_wifi(self, ssid='TELLO-XXXXXX'):
        # List of available Wi-Fi networks
        result = subprocess.run(['nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'], capture_output=True, text=True)
        redes = result.stdout.splitlines()
        if ssid in redes:
            # Try to connect to the Tello Wi-Fi network
            subprocess.run(['nmcli', 'dev', 'wifi', 'connect', ssid], check=True)
            print(f'Conectado à rede {ssid}')
        else:
            print(f'Rede {ssid} não encontrada!')

    def publish_video_stream(self):

        try:
            frame = self.frame_read.frame
            if frame is not None:
                # Convert the OpenCV image to a ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'tello_camera'
                self.video_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing video stream: {e}')

    def takeoff_callback(self, request, response):
        try:
            self.tello.takeoff()
            response.success = True
            response.message = 'Takeoff successful'
        except Exception as e:
            response.success = False
            response.message = f'Takeoff failed: {e}'
        return response
    
    def land_callback(self, request, response):
        try:
            self.tello.land()
            response.success = True
            response.message = 'Landing successful'
        except Exception as e:
            response.success = False
            response.message = f'Landing failed: {e}'
        return response

    def cmd_vel_callback(self, msg):
        try:
            # Convert Twist message to Tello command and clamp values between -100 and 100
            linear_x = max(-100, min(100, int(msg.linear.x * 100)))
            linear_y = max(-100, min(100, int(msg.linear.y * 100)))
            linear_z = max(-100, min(100, int(msg.linear.z * 100)))
            angular_z = max(-100, min(100, int(msg.angular.z * 100)))

            self.tello.send_rc_control(linear_y, linear_x, linear_z, angular_z)
        except Exception as e:
            self.get_logger().error(f'Error processing cmd_vel: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tello.end()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
