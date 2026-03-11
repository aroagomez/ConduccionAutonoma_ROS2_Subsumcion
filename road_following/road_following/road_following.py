#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class RoadFollowingNode(Node):
    def __init__(self):
        super().__init__('road_following_node')
        # Get package path
        self.package_path = get_package_share_directory('road_following')
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ground_image_topic', '/road_perception/ground'), 
                ('cmd_vel_topic', '/cmd_vel')
            ]
        )

        # Get parameters
        ground_image_topic = self.get_parameter('ground_image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # ROS Subscribers
        ground_image_sub = self.create_subscription(Image, ground_image_topic, self.image_callback, 10)
        
        # ROS Publishers
        self.velocity_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.blancos_publisher= self.create_publisher(Int32MultiArray, "blancos", 10)
        
        # OpenCV bridge
        self.bridge = CvBridge()

        # Behavior variables

        
        # Node started message
        self.get_logger().info("Road Following Node started")
        self.get_logger().info('\nLaunch parameters: \n\t> ground_image_topic: %s \n\t> cmd_vel_topic: %s' % (ground_image_topic, cmd_vel_topic))

    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            image_width = cv_image.shape[1]
            image_height = cv_image.shape[0]
            # Print image information
            # self.get_logger().info(f"Image width (columns): {image_width}, Image height (rows): {image_height}")
            # Pixel info
            # self.get_logger().info(f"Pixel info: {cv_image[image_width//2, image_height//2]}")
            
            # TODO: The robot must be in the middle of the road
            linear_velocity = 0.2
            angular_velocity = 0.0
            #tamaño imagen 320*240

            # col_mas_alta = 0
            # blancos_col_mas_alta = 0

            # blanco = np.array([255, 255, 255])
            # negro = np.array([0, 0, 0])
            # self.get_logger().info(f"Blanco info: {blanco}")
            # for i in range(image_width): #cambio de columnas
            #     blancos_col = 0
            #     for j in reversed(range(image_height)): #cambio de filas
            #         if np.array_equal(cv_image[j, i], blanco):
            #             blancos_col += 1
            #         if np.array_equal(cv_image[j, i], negro):
            #             break
            #     if blancos_col > blancos_col_mas_alta:
            #         col_mas_alta = i
            #         blancos_col_mas_alta = blancos_col
            # #girar en proporcion a lo a la izquierda o derecha que esté la más alta, para tener la más alta en el centro
            # angular_velocity = 0.5*((image_width//2 - col_mas_alta) // (image_width//2))


           
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

            # # Obtenemos la columna con más blancos consecutivos
            col_mas_alta = np.argmax(blancos_col_counts)
            blancos_col_mas_alta = blancos_col_counts[col_mas_alta]

            # self.get_logger().info(f"colmasalta: {col_mas_alta}")
            # self.get_logger().info(f"blancos_col_mas_alta: {blancos_col_mas_alta}")

            blancos_msg = Int32MultiArray()
            blancos_msg.data = [int(col_mas_alta), int(blancos_col_mas_alta)]
            self.blancos_publisher.publish(blancos_msg)


            # Calculamos velocidad angular
            angular_velocity = 0.4 * ((image_width // 2 - col_mas_alta) / (image_width // 2))
            if angular_velocity > 0.1:
                linear_velocity = 0.05*abs(angular_velocity)
            else:
                linear_velocity = 0.2

            if blancos_col_mas_alta < 100:
                angular_velocity = -0.1
                linear_velocity = 0.0


            # End TODO
            
            # Publish velocity commands
            twist_msg = Twist()
            twist_msg.linear.x = min(linear_velocity, 0.3)
            twist_msg.angular.z = np.clip(angular_velocity, -0.5, 0.5)
            self.velocity_publisher.publish(twist_msg)

            # Opencv image show
            cv2.imshow("Road Following", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    road_following_node = RoadFollowingNode()
    try:
        rclpy.spin(road_following_node)
    except KeyboardInterrupt:
        road_following_node.get_logger().info("Node interrupted by keyboard (CTRL+C)")
    finally:
        road_following_node.destroy_node()  
        rclpy.shutdown()

if __name__ == '__main__':
    main()
