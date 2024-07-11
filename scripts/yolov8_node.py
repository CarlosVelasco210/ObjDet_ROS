#!/usr/bin/env python3

import rospy
import cv_bridge
import roslib.packages

from ultralytics import YOLO
from sensor_msgs.msg import Image

from yolov8.msg import BoundingBox2D
from yolov8.msg import Point2D
from yolov8.msg import Mask
from yolov8.msg import KeyPoint2DArray
from yolov8.msg import KeyPoint2D
from yolov8.msg import DetectionArray
from yolov8.msg import Detection

from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

class YoloV8Node:
    def __init__(self):
        
        path = roslib.packages.get_pkg_dir("ultralytics_ros")
        self.bridge = cv_bridge.CvBridge()

        #param
        image_topic = rospy.get_param("~input_topic")
        yolo_model = rospy.get_param("~yolo_model")
        self.conf_thres = rospy.get_param("~conf_thres")
        self.model = YOLO(f"{path}/models/{yolo_model}")
        
        #pub
        self._pub = rospy.Publisher("detections", DetectionArray, queue_size=10)
        
        #sub
        self._sub = rospy.Subscriber(
            image_topic, Image, self.image_cb, queue_size=10
        )
    
        #Desgloce de las imagenes a detectar
    def image_cb(self, msg):
        # convert image + predict
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.conf_thres,
            device=0
        )
        results: Results = results[0].cuda()

        if results.boxes:
            #Deteccion de clases , nombre e ID
            hypothesis = self.parse_hypothesis(results)
            #Deteccion de los cuadros que se detecta en la imagen
            boxes = self.parse_boxes(results)

        if results.masks:
            #Deteccion de mascaras
            masks = self.parse_masks(results)

        if results.keypoints:
            #Deteccion de keypoints de los objetos
            #Solo sirve para el modelo yolov8-pose
            keypoints = self.parse_keypoints(results)

        # create detection msgs
        detections_msg = DetectionArray()

        for i in range(len(results)):
            
            #Crea una variable msg de tipo Detection donde se guardaran las detecciones
            aux_msg = Detection()

            if results.boxes:
                aux_msg.class_id = hypothesis[i]["class_id"]
                aux_msg.class_name = hypothesis[i]["class_name"]
                aux_msg.score = hypothesis[i]["score"]

                aux_msg.bbox = boxes[i]

            if results.masks:
                aux_msg.mask = masks[i]

            if results.keypoints:
                aux_msg.keypoints = keypoints[i]

            # Una vez asignado los nombres de clases, ID, score, masks, box y keypoints con
            # su respectivo objeto se guarda en la lista detections_msg
            detections_msg.detections.append(aux_msg)

        # publish detections
        detections_msg.header = msg.header
        self._pub.publish(detections_msg)
        
    def parse_hypothesis(self, results: Results):

        hypothesis_list = []

        box_data: Boxes
        # Recorre cada deteccion y guarda la ID, nombre y score en una lista
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.model.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            # Guardando en la lista
            hypothesis_list.append(hypothesis)

        return hypothesis_list
    
    def parse_boxes(self, results: Results):

        boxes_list = []

        box_data: Boxes
        # Recorre cada deteccion y guarda el cuadro delimitdor en msg de tipo BoundingBox2D
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            # Tamño de alto y ancho
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list      
    
    def parse_masks(self, results: Results):

        masks_list = []

        # Funcion para guardar los Point2D detectados en msg de tipo Point2D
        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        # Recorre cada deteccion y guarda las mascaras detectadas en msg de tipo Mask
        for mask in results.masks:

            msg = Mask()

            #Los puntos de la mascara que se detecto en los objetos
            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list
    
    #Esto se utiliza con el modelo yolov8-pose
    def parse_keypoints(self, results: Results):

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:
            msg_array = KeyPoint2DArray()
            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.conf_thres:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

if __name__ == "__main__":
    rospy.init_node("yolov8_node")
    node = YoloV8Node()
    rospy.spin()