#!/usr/bin/env python3

"""
Nodo ROS2 para realizar emparejamiento de puntos clave, aplicar RANSAC y publicar una nube de puntos 3D.
"""

# Importaciones de ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs

# Importaciones estándar
import yaml
import numpy as np
import cv2


class MatcherNode(Node):
    """Nodo para procesar imágenes estéreo y generar nubes de puntos 3D."""

    def __init__(self):
        super().__init__('matcher_node')

        # Publicadores
        self.matches_pub = self.create_publisher(Image, 'matches', 10)
        self.point_cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.right_image_with_left = self.create_publisher(Image, '/right/image_transformed', 10)

        # Suscriptores sincronizados para las imágenes rectificadas
        self.left_image_sub = Subscriber(self, Image, '/left/image_rect_color')
        self.right_image_sub = Subscriber(self, Image, '/right/image_rect_color')

        # Sincronizador para las imágenes izquierda y derecha
        self.sync = ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.__callback)

        # Inicializar herramientas de procesamiento
        self.br = CvBridge()
        self.kpe = cv2.ORB_create()  # Detector ORB
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # Matcher

        # Cargar datos de calibración
        self.load_calibration_data()
        self.get_logger().info("MatcherNode inicializado y listo para procesar.")

    def load_calibration_data(self):
        """
        Carga los datos de calibración desde archivos YAML.
        """
        with open('calibrationdata/left.yaml', 'r') as file:
            left_data = yaml.safe_load(file)

        with open('calibrationdata/right.yaml', 'r') as file:
            right_data = yaml.safe_load(file)

        # Extraer matrices de proyección de las cámaras
        self.left_proj = np.array(left_data['projection_matrix']['data']).reshape((3, 4))
        self.right_proj = np.array(right_data['projection_matrix']['data']).reshape((3, 4))
        self.get_logger().info("Datos de calibración cargados.")

    def __callback(self, left_msg, right_msg):
        """
        Callback sincronizado que procesa las imágenes izquierda y derecha.
        """
        try:
            self.__process(left_msg, right_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Error en CvBridge: {e}')
        except Exception as e:
            self.get_logger().error(f'Error no manejado: {e}')

    def __process(self, left_msg: Image, right_msg: Image):
        """
        Procesa las imágenes para encontrar puntos clave, aplicar RANSAC y generar una nube de puntos 3D.
        """
        # Convertir mensajes de imagen ROS a imágenes OpenCV
        left_frame = self.br.imgmsg_to_cv2(left_msg)
        right_frame = self.br.imgmsg_to_cv2(right_msg)

        # Detectar puntos clave y descriptores
        left_kp, left_desc = self.kpe.detectAndCompute(left_frame, None)
        right_kp, right_desc = self.kpe.detectAndCompute(right_frame, None)

        if left_desc is None or right_desc is None:
            self.get_logger().warning("No se encontraron puntos clave.")
            return

        # Emparejar descriptores con fuerza bruta
        matches = self.bf.match(left_desc, right_desc)
        good_matches = [m for m in matches if m.distance < 30]

        if not good_matches:
            self.get_logger().warning("No se encontraron buenos matches.")
            return
        
        # Para evitar el error el siguiente error: 
        # The input arrays should have at least 4 corresponding point sets to calculate Homography in function 'findHomography'
        if len(good_matches) < 4:
            self.get_logger().warning("No hay suficientes matches para calcular la homografía.")
            return
        
        # Aplicar RANSAC para encontrar la matriz homográfica
        left_pts = np.float32([left_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        right_pts = np.float32([right_kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(left_pts, right_pts, cv2.RANSAC, 5.0)
        ransac_matches = [good_matches[i] for i in range(len(mask)) if mask[i]]

        # Dibujar y publicar los matches filtrados
        match_img = cv2.drawMatches(
            left_frame, left_kp, right_frame, right_kp, ransac_matches, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        self.matches_pub.publish(self.br.cv2_to_imgmsg(match_img, encoding="bgr8"))

        # Triangular puntos 3D
        left_pts_ransac = np.float32([left_kp[m.queryIdx].pt for m in ransac_matches]).T
        right_pts_ransac = np.float32([right_kp[m.trainIdx].pt for m in ransac_matches]).T

        if left_pts_ransac.shape[1] < 2 or right_pts_ransac.shape[1] < 2:
            self.get_logger().warning("No hay suficientes puntos para triangular.")
            return

        triang_points_hom = cv2.triangulatePoints(self.left_proj, self.right_proj, left_pts_ransac, right_pts_ransac)
        triang_points_euc = cv2.convertPointsFromHomogeneous(triang_points_hom.T).reshape(-1, 3)

        # Publicar nube de puntos 3D
        self.publish_point_cloud(triang_points_euc)

        # Transformar puntos y publicar en la imagen derecha
        self.visualize_transformed_points(right_frame, left_pts, H)

    def publish_point_cloud(self, points: np.ndarray):
        """
        Publica una nube de puntos 3D.
        """
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "body"
        point_cloud_msg = pc2.create_cloud(header, fields, points)
        self.point_cloud_pub.publish(point_cloud_msg)

    def visualize_transformed_points(self, right_frame: np.ndarray, left_pts: np.ndarray, H: np.ndarray):
        """
        Transforma y visualiza puntos de la imagen izquierda en la derecha.
        """
        left_pts_transformed = cv2.perspectiveTransform(left_pts, H)
        right_frame_bgr = cv2.cvtColor(right_frame, cv2.COLOR_GRAY2BGR)

        for pt in left_pts_transformed:
            cv2.circle(right_frame_bgr, tuple(int(x) for x in pt[0]), radius=5, color=(0, 255, 0), thickness=-1)
        self.right_image_with_left.publish(self.br.cv2_to_imgmsg(right_frame_bgr, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = MatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Finalizando MatcherNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
