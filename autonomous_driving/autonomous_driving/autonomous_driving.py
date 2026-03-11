#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math as m
from cv_bridge import CvBridge
import cv2
import time

from std_msgs.msg import Int32
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from road_perception.msg import TrafficSignArray
from std_msgs.msg import Int32MultiArray


# Define global constants for traffic sign labels
SIGN_FORWARD = 0
SIGN_LEFT = 1
SIGN_RIGHT = 2
SIGN_STOP = 3
SIGN_EMPTY = -1

class AutonomousDrivingNode(Node):
    def __init__(self):
        super().__init__('autonomous_driving_node')
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('traffic_signs_topic', '/road_perception/traffic_signs'),
                ('road_following_vel_topic', '/road_following/cmd_vel'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('road_perception_ground_topic', '/road_perception/ground'),
                ('odom_topic', '/odom'),
                ('scan_topic', '/scan'),
                ('lidar_type', 's2'),
                ('simulation', True)
            ])
        # Get parameters
        traffic_signs_topic = self.get_parameter('traffic_signs_topic').value
        road_follow_velocity_topic = self.get_parameter('road_following_vel_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        road_perception_ground_topic = self.get_parameter('road_perception_ground_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic_ = self.get_parameter('scan_topic').value
        self.lidar_type = self.get_parameter('lidar_type').value
        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value

        # Variables for callbacks
        self.pose = [0, 0, 0]  # x, y, yaw
        self.front_scan = []
        self.scan_uninitialized = True
        self.sign_lbl = SIGN_EMPTY
        self.blancos_data = None

        self.senal_anterior = SIGN_EMPTY

        self.seguir_girando = False
        self.girando = False
        self.stop_hecho = True
        self.tiempo_inicial_stop = 0.0
        self.contador_sign_vista = 0

        self.road_following_vel = Twist()
        self.giro_vel = Twist()
        self.vel_msg = Twist()

        # Used for image processing
        self.bridge = CvBridge()


        # Subscribers
        road_ground_sub = self.create_subscription(Image, road_perception_ground_topic, self.ground_image_callback, 10)
        road_following_vel_sub = self.create_subscription(Twist, road_follow_velocity_topic, self.follow_road_callback, 10)
        traffic_sign_sub = self.create_subscription(TrafficSignArray, traffic_signs_topic, self.traffic_signs_callback, 10)
        odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        scan_sub = self.create_subscription(LaserScan, scan_topic_, self.lidar_callback, 10)
        blancos_sub = self.create_subscription(Int32MultiArray, "blancos", self.blancos_callback, 10)
        # Publishers
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        # self.image_pub = self.create_publisher()
        # Control Architecture
        self.create_timer(0.1, self.subsumption_architecture)

        # Print parameters
        self.get_logger().info(f"Simulation mode: {self.simulation}")

        self.get_logger().info(f">>>> Self Driving Node initialized <<<<")

    ##################################### Callbacks #####################################
        
    def blancos_callback(self, msg):
        self.blancos_data = np.array(msg.data, dtype=np.int32)
        # self.get_logger().info(f">>>>>>>>>>>>>> {self.blancos_data}")
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)

        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        self.pose[2] = yaw
        # self.get_logger().info(f"Odom Pose -> X: {self.pose[0]:.2f}, Y: {self.pose[1]:.2f}, Yaw: {m.degrees(self.pose[2]):.2f} deg")

    def lidar_callback(self, msg):            
        self.scan = msg.ranges
        if self.scan_uninitialized:
            self.scan_count = len(msg.ranges)
            self.max_range = msg.range_max
            self.min_range = msg.range_min
            self.scan_mid = self.scan_count // 2
            self.laser_Q1 = self.scan_mid//2
            self.laser_Q2 = self.scan_mid
            self.laser_Q3 = self.scan_mid//2 + self.scan_mid
            # self.get_logger().info("\n Laser info: \n \t > max_range: %.2f \n \t > scan_count: %d \n \t > anglemin: %.2f \n \t > angleinc: %.5f"%(scan_msg.range_max, self.scan_count, scan_msg.angle_min, scan_msg.angle_increment))
            # Compute bearings
            self.bearings = []
            for i in range(0, self.scan_count):
                angle = msg.angle_min + msg.angle_increment*i
                self.bearings.append(angle)
            self.scan_uninitialized = False
        
        if not self.scan_uninitialized:
            # Replace infinity values with max_range 
            self.scan = [x if x < self.max_range else self.max_range for x in msg.ranges]
            
            # Reorganize scan indexes to make it easier to work with. 
            # 0 index corresponds to the back side of the robot for both, scan and bearings
            if self.lidar_type == "s2":
                self.scan = [self.scan[i - self.scan_mid] for i in range(self.scan_count)]
            if self.lidar_type == "a1":
                self.scan = [self.scan[self.scan_mid-i] for i in range(self.scan_count)]
            
            # Filter scan to front only (180 degrees)
    
            for i in range(self.laser_Q1, self.laser_Q3):
                self.front_scan.append(self.scan[i])


            # Print scan len for debugging, remove later
            # self.get_logger().info(f"Lidar Front Scan Length: {len(self.front_scan)}")


    def traffic_signs_callback(self, msg):

        # self.get_logger().info(f"INICIO GIRO. Yaw inicial: {self.angulo_inicial_giro:.2f}")
        # Get the signal with the largest BBox
        umbral_senal_vista = 6

        if msg.signs:
            max_sign = max(msg.signs, key=lambda s: s.bbox.size_x * s.bbox.size_y)
            self.sign_lbl = int(max_sign.class_num)
        else:
            self.sign_lbl = SIGN_EMPTY


        if self.sign_lbl == SIGN_LEFT and not self.seguir_girando and self.contador_sign_vista > umbral_senal_vista:

            self.senal_anterior = SIGN_LEFT
            self.seguir_girando = True
            self.contador_sign_vista = 0
            self.angulo_inicial_giro = self.pose[2]

        elif self.sign_lbl == SIGN_RIGHT and not self.seguir_girando and self.contador_sign_vista > umbral_senal_vista:

            self.senal_anterior = SIGN_RIGHT
            self.seguir_girando = True
            self.contador_sign_vista = 0
            self.angulo_inicial_giro = self.pose[2]

        elif self.sign_lbl == SIGN_FORWARD and self.contador_sign_vista > umbral_senal_vista:

            self.senal_anterior = SIGN_FORWARD
            self.contador_sign_vista = 0

        elif self.sign_lbl == SIGN_STOP and self.contador_sign_vista > umbral_senal_vista:

            self.senal_anterior = SIGN_STOP
            self.tiempo_inicial_stop = time.time()
            self.stop_hecho = False
            self.contador_sign_vista = 0

        elif ((self.sign_lbl == SIGN_LEFT and not self.seguir_girando) or (self.sign_lbl == SIGN_RIGHT and not self.seguir_girando) or (self.sign_lbl == SIGN_FORWARD) or (self.sign_lbl == SIGN_STOP)) and self.senal_anterior == SIGN_EMPTY:

            self.contador_sign_vista += 1

        # Print detected sign label for debugging, remove later
        # self.get_logger().info(f"Detected Traffic Sign Label: {self.sign_lbl}")
        

    def get_col_ref(self, cv_image):

        blanco = np.array([255, 255, 255])
        negro = np.array([0, 0, 0])
  
        mask_blanco = np.all(cv_image == blanco, axis=2)
        mask_negro = np.all(cv_image == negro, axis=2)

        image_height, image_width, _ = cv_image.shape

        # Para cada columna, encontramos cuántos píxeles blancos hay desde abajo hacia arriba
        # hasta el primer píxel negro.
        blancos_col_counts = np.zeros(image_width, dtype=int)

        for i in range(image_width):
            col_blancos = mask_blanco[:, i]
            col_negros = mask_negro[:, i]
            
            # Invertimos la columna (para contar desde abajo)
            col_blancos = col_blancos[::-1]
            col_negros = col_negros[::-1]
            
            # Encontramos el índice del primer negro
            negros_idx = np.where(col_negros)[0]
            limite = negros_idx[0] if len(negros_idx) > 0 else len(col_blancos)
            
            # Contamos cuántos blancos hay antes del primer negro
            blancos_col_counts[i] = np.sum(col_blancos[:limite])

        blancos_col_izda = np.sum(mask_blanco[:, 0])
        blancos_col_dcha = np.sum(mask_blanco[:, image_width - 1])

        # # Obtenemos la columna con más blancos consecutivos
        col_mas_alta = np.argmax(blancos_col_counts)

        return col_mas_alta, blancos_col_dcha, blancos_col_izda

    def ground_image_callback(self, msg):
        
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        image_height, image_width, _ = cv_image.shape

        if self.seguir_girando:
            
            if self.senal_anterior == SIGN_LEFT:
                # Si hay que ir a la izquierda, tapamos la derecha
                # Pintamos de negro desde la mitad hasta el final del ancho
                cv_image[:, image_width//4:] = 0 
                
            elif self.senal_anterior == SIGN_RIGHT:
                # Si hay que ir a la derecha, tapamos la izquierda
                # Pintamos de negro desde el inicio hasta la mitad del ancho
                cv_image[:, : 3 * image_width//4] = 0

            col_mas_alta, blancos_col_dcha, blancos_col_izda = self.get_col_ref(cv_image)
            # self.get_logger().info(f"COL MAS ALTA: {col_mas_alta}")
            # Calculamos velocidad angular
            angular_velocity = 0.0
            self.get_logger().info(f">>>>>>>>>>BLANCOS IZDA: {blancos_col_izda}")
            self.get_logger().info(f">>>>>>>>>>BLANCOS DCHA: {blancos_col_dcha}")
            if blancos_col_dcha > 124 or blancos_col_izda > 124 or self.girando:
                self.girando = True
                angular_velocity = 0.4 * ((image_width // 2 - col_mas_alta) / (image_width // 2))
            if angular_velocity > 0.1:
                linear_velocity = 0.05*abs(angular_velocity)
            else:
                linear_velocity = 0.2

            self.giro_vel.linear.x = linear_velocity
            self.giro_vel.angular.z = angular_velocity

        # Publicamos la imagen 
        cv2.imshow("Autonomous Image", cv_image)
        cv2.waitKey(1)

        
    def follow_road_callback(self, msg):
        self.road_following_vel = msg
        # Print received velocity for debugging, remove later
        # self.get_logger().info(f"Received Road Following Velocity -> Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}")

    def subsumption_architecture(self):
        tiempo_stop = 5.0
        
        # PRIORIDAD MÁXIMA: STOP
        if self.senal_anterior == SIGN_STOP and not self.stop_hecho:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            self.get_logger().info("STOP")

            if (time.time() - self.tiempo_inicial_stop) >= tiempo_stop:
                self.stop_hecho = True
                self.senal_anterior = SIGN_EMPTY
                self.get_logger().info("STOP finalizado")

        # PRIORIDAD MEDIA: GIRO 
        elif self.seguir_girando:
            self.vel_msg = self.giro_vel
            self.get_logger().info("Girando")
            self.get_logger().info(f">>>>>>>>>>>>>>>>>>Contador senal vista: {self.contador_sign_vista}" )
            
            dif_angulo = abs(self.pose[2] - self.angulo_inicial_giro)
            self.get_logger().info(f">> DIF ANGULO: {dif_angulo}")
            if (dif_angulo > 1.57):
                self.seguir_girando = False
                self.girando = False
                self.senal_anterior = SIGN_EMPTY
                self.get_logger().info("Giro finalizado")

        # PRIORIDAD BAJA: SEGUIR CARRETERA (Normal)
        else:
            self.vel_msg = self.road_following_vel
        
        # Publish the final velocity command
        self.vel_pub.publish(self.vel_msg)

# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDrivingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()