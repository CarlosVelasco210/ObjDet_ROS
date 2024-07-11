#!/usr/bin/env python3

import rospy

import geometry_msgs.msg
import tf2_msgs.msg
import tf2_ros
import tf
import message_filters

from geometry_msgs.msg import PoseStamped
from yolov8.msg import DetectionArray

class TransformPublish:
    def __init__(self):
        self.stampped = geometry_msgs.msg.TransformStamped()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        #self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        self.detection = message_filters.Subscriber(
            "detections_3D", DetectionArray
        )

        self.posedetect = message_filters.Subscriber(
            '/dope/pose_red_block', PoseStamped
        )

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.posedetect, self.detection], 10, 0.5)
        
        self._synchronizer.registerCallback(self.PublishTransform)

    def PublishTransform(self, pose_msg: PoseStamped, detection_msg: DetectionArray):

        #print(pose_msg.pose.orientation)
        Dope_orientation = pose_msg.pose.orientation
        
        detections = detection_msg.detections
        for detection in detections:
            box3d = detection.bbox3d
            center = box3d.center.position
            #orientation = box3d.center.orientation

            stampped = geometry_msgs.msg.TransformStamped()

            stampped.header.frame_id = box3d.frame_id
            #print(box3d.frame_id)
            stampped.header.stamp = rospy.Time.now()
            stampped.child_frame_id = 'textured.obj_1'

            stampped.transform.translation.x = center.x
            stampped.transform.translation.y = center.y    
            stampped.transform.translation.z = center.z

            #quat = tf.transformations.quaternion_from_euler(0, 2, -1.6)

            stampped.transform.rotation.x = Dope_orientation.x
            stampped.transform.rotation.y = Dope_orientation.y
            stampped.transform.rotation.z = Dope_orientation.z
            stampped.transform.rotation.w = Dope_orientation.w

            self.tf_broadcaster.sendTransform(stampped)

            #tfm = tf2_msgs.msg.TFMessage([self.stampped])
            #self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node("transform_publish")
    node = TransformPublish()
    rospy.spin()