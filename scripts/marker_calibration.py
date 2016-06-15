#!/usr/bin/env python

# marker_calibration.py: Code to calibrate kinect with baxter
# Author: Nishanth Koganti
# Date: 2016/06/10
# Source: https://github.com/osrf/baxter_demos/blob/master/scripts/get_ar_calib.py

import tf
import yaml
import math
import rospy
import numpy as np
from math import pi

# get tf details from transformation matrix
def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles), angles

# lookup tf transform between two frames
def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))

    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)

    source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
    return source_target

def main():
    # initialize ros node
    rospy.init_node('marker_calibration')

    # load calibration file
    saveFile = rospy.get_param('~save_file')
    paramFile = rospy.get_param('~parameters_file')

    with open(paramFile, 'r') as f:
        params = yaml.load(f)

    # parameter initialization
    frame = params['frame']
    trans = params['translation']
    markernum = params['markernum']
    theta = params['theta']*math.pi/180.0
    rot = np.array([1, 0, 0, 0, math.cos(theta), math.sin(theta), 0, -math.sin(theta), math.cos(theta)]).reshape(3,3)

    # create tf listener and broadcaster
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # loop rate
    rate = rospy.Rate(100)

    # calculate transform from reference frame to marker
    reference_marker_trans = np.dot(np.linalg.inv(rot),-np.array(trans).reshape((3, 1))).flatten()
    reference_marker = tf.transformations.compose_matrix(translate = reference_marker_trans, angles = tf.transformations.euler_from_matrix(rot))

    def saveCalibration():
        # write to yaml file
        print 'write transform to yaml file'
        f = open(saveFile, 'w')
        lines = ['trans: [', 'rot: [', 'rot_euler: [']

        for elem in trans: lines[0] += str(elem) + ', '
        lines[0] += ']\n'
        for elem in rot: lines[1] += str(elem) + ', '
        lines[1] += ']\n'
        for elem in rot_euler: lines[2] += str(elem) + ', '
        lines[2] += ']\n'

        lines.append('parent: /base\n')
        lines.append('child: /kinect2_link\n')

        f.writelines(lines)
        f.close()

    rospy.on_shutdown(saveCalibration)

    # Publish transform and marker
    while not rospy.is_shutdown():

        # base to refernce
        base_reference = lookupTransform(tf_listener, frame, '/base')

        # base to marker = reference to marker * base to reference
        base_marker = reference_marker.dot(base_reference)
        trans1, rot1, rot_euler1 = getTfFromMatrix(np.linalg.inv(base_marker))
        tf_broadcaster.sendTransform(trans1, rot1, rospy.Time.now(), '/ar_marker_'+str(markernum), '/base')

        # marker to camera
        marker_camera = lookupTransform(tf_listener, '/kinect2_link', '/ar_marker_'+str(markernum))

        # base to camera = marker to camera * base to marker
        base_camera = marker_camera.dot(base_marker)
        trans, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_camera))
        tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), '/kinect2_link', '/base')

        rate.sleep()

if __name__=='__main__':
    main()
