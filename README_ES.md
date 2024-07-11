Español | [English](README.md)
# Detección de Objetos y Localización 3D con YOLOv8 en ROS
## Descripción
Este proyecto está basado en yolov8_ros creado por mgonzs13 y ha sido modificado para poder utilizarlo en ROS Noetic Ninjemyz. Está diseñado para la detección y localización de objetos utilizando Ultralytics YOLOv8 con imágenes de profundidad.
El proyecto ha sido probado con un Kinect V1 y un Kinect V2. Para el uso de Kinect V2, fue necesario instalar el repositorio iai_kinect2, el cual proporciona herramientas y librerías específicas para su integración con ROS

https://github.com/CarlosVelasco210/DetObj_ROS/assets/69257527/91c92260-9149-4b85-bc3c-493b62194b34

## Requisitos:
- Ultralytics YoloV8
- Iai_kinectV2
- Ros Noetic Ninjemyz
## Tópicos:
- /yolov8/dbg_image: Imágenes depuradas que muestran los objetos detectados y rastreados. Se pueden visualizar en Rviz.
- /yolov8/pub_tf: Publica la posición del objeto como un transform en el espacio tridimensional. Permite la visualización de la posición del objeto en Rviz.
- /yolov8/detections: Objetos detectados por YOLOv8 usando las imágenes RGB proporcionados por el dispositivo a utilizar, cada objeto contiene una caja delimitadora y un nombre de clase.
## Parámetros:
- yolo_model: Modelo YOLOv8 a utilizar, en este caso se realizó un entrenamiento para utilizar un modelo personalizado con el objeto a detectar que utilizamos
- conf_tresh: Configuración del umbral para la detección (Valor por defecto: 0.5)
- input_topic: El tópico de la cámara para imágenes RGB
- depth_image: El tópico que proporciona imágenes de profundidad
- camera_depth_info: El tópico de la cámara que proporciona información relacionada con la profundidad.
## Nota:
El proyecto está configurado para ser utilizado con un Kinect V2, si desea utilizar su propio dispositivo o cámara, puede modificar los parámetros en el archivo yolo.launch según sus requisitos.
## Instalación:
``` bash
cd <catkin_ws/src>
git clone https://github.com/CarlosVelasco210/DetObj_ROS.git
cd  ~/catkin_ws
catkin_make
```
## Ejecutar:
```
roslaunch yolov8 yolo.launch
```
## Cita
```tex
@misc{iai_kinect2,
  author = {Wiedemeyer, Thiemo},
  title = {{IAI Kinect2}},
  organization = {Institute for Artificial Intelligence},
  address = {University Bremen},
  year = {2014 -- 2015},
  howpublished = {\url{https://github.com/code-iai/iai\_kinect2}},
  note = {Accessed June 12, 2015}
}
```
