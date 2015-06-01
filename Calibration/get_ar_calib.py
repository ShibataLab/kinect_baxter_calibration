#!/usr/bin/env python

import tf
import yaml
import numpy
import rospy
from math import pi
import visualization_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

with open('ar_calib.yaml', 'r') as f:
    params = yaml.load(f)

class markerSubscriber():
    def __init__(self, markernum=2):
        self.sub = rospy.Subscriber("/visualization_marker", Marker, self.callback)
        self.markernum = markernum
        self.pose = None
    def callback(self, data):
        self.pose = data.pose

def getPoseFromMatrix(matrix):
    trans, quat = getTfFromMatrix(numpy.linalg.inv(matrix))
    return Pose(position=Point(*trans), orientation=Quaternion(*quat))

def getMatrixFromPose(pose):
    trans = (pose.position.x, pose.position.y, pose.position.z )
    rot = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    return tf.transformations.compose_matrix(translate = trans, angles = rot)

def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles),angles

def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))
    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)
    source_target = tf.transformations.compose_matrix(translate = trans,
                                                     angles = euler)
    #print "looked up transform from", source, "to", target, "-", source_target
    return source_target

def create_marker(ns, id_num, shape_type, pose, color, scale):
    # Create rviz marker message
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.ns = ns
    marker.id = id_num
    marker.type = shape_type
    marker.action = Marker.ADD
    marker.pose = pose
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    return marker

markernum = params['markernum']
measured_translation = params['measured_translation']
measured_rot = numpy.array(params['measured_rot']).reshape((3,3))
frame = params['frame']
squaredims = tuple(params['squaredims'])

rospy.init_node("get_ar_calib")

tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

marker_pub = rospy.Publisher("ar_calib_markers", MarkerArray)

rate = rospy.Rate(100)

reference_marker_trans = numpy.dot(numpy.linalg.inv(measured_rot),
                               -numpy.array(measured_translation).reshape((3, 1))).flatten()
print reference_marker_trans
# Calculate transform from forearm to marker
reference_marker = tf.transformations.compose_matrix(
                translate = reference_marker_trans,
                angles = tf.transformations.euler_from_matrix(measured_rot))
marker_sub = markerSubscriber()

# Publish transform and marker
while not rospy.is_shutdown():

    # base to forearm
    base_reference = lookupTransform(tf_listener, frame, '/base')

    # Compose transforms
    # base to marker = forearm to marker * base to forearm
    base_marker = reference_marker.dot(base_reference)
    trans, rot, rot_euler = getTfFromMatrix(numpy.linalg.inv(base_marker))
    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), '/ar_marker_'+str(markernum), '/base')
    marker_pose = getPoseFromMatrix(base_marker)

    # marker to camera
    marker_camera = lookupTransform(tf_listener, '/kinect2_link', '/ar_marker_'+str(markernum))

    # base to camera = marker to camera * base to marker
    base_camera = marker_camera.dot(base_marker)
    trans, rot = getTfFromMatrix(numpy.linalg.inv(base_camera))

    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/kinect2_link", "/base")
    camera_pose = getPoseFromMatrix(base_camera)

    marker_msg = create_marker("marker_pose", 44, Marker.CUBE, marker_pose, (0, 255, 0), squaredims )

    camera_msg = create_marker("camera_pose", 1337, Marker.ARROW, camera_pose, (0, 0, 255), (0.2, 0.01, 0.01))

    # Now refine the transform using least squares
    # Get the pose of the marker from ar_pose_marker in the camera frame
    msg = [marker_msg, camera_msg]

    marker_pub.publish(msg)
    rate.sleep()

print "Writing transform to yaml file"
# Write to yaml file
f = open("base_camera_tf.yaml", 'w')
lines = ['trans: [', 'rot: [', 'rot_euler: [']
for elem in trans:
    lines[0] += str(elem) + ', '
lines[0] += ']\n'
for elem in rot:
    lines[1] += str(elem) + ', '
lines[1] += ']\n'
for elem in rot_euler:
    lines[2] += str(elem) + ', '
lines[2] += ']\n'
lines.append('parent: /base\n')
lines.append('child: /kinect2_link\n')
f.writelines(lines)
f.close()
