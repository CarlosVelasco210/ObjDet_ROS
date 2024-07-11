#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

# Este script se utilizo con el fin de posicionarl el kinect2 correctamente en el espacio tridimensional de Rviz, para 
# realizar las pruebas de manipulacion y deteccion

if __name__ == '__main__':
    rospy.init_node('static_kinect')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base"
    static_transformStamped.child_frame_id = "kinect2_link"

    static_transformStamped.transform.translation.x = 1
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 1

    q = tf_conversions.transformations.quaternion_from_euler(-1.5, 0, 1.6)

    static_transformStamped.transform.rotation.x = q[0]
    static_transformStamped.transform.rotation.y = q[1]
    static_transformStamped.transform.rotation.z = q[2]
    static_transformStamped.transform.rotation.w = q[3]

    broadcaster.sendTransform(static_transformStamped)
    
    rospy.spin()