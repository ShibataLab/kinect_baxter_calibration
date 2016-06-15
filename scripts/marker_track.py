#!/usr/bin/env python

# marker_track.py: Code to track AR marker with respect to Kinect and Baxter
# Author: Nishanth Koganti
# Date: 2016/06/15
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
    rospy.init_node('marker_track')

    # load calibration file
    savePath = rospy.get_param('~save_path')
    paramFile = rospy.get_param('~parameters_file')

    with open(paramFile, 'r') as f:
        params = yaml.load(f)

    # parameter initialization
    frame = params['frame']
    markernum = params['markernum']

    # create tf listener and broadcaster
    tf_listener = tf.TransformListener()

    # loop rate
    rate = rospy.Rate(30)

    def saveData():
        # write to yaml file
        print 'write tracking data to files'
        np.savetxt('%sbaxter_trajectory' % (savePath), baxterTraj, delimiter=',', fmt='%.4f')
        np.savetxt('%skinect_trajectory' % (savePath), kinectTraj, delimiter=',', fmt='%.4f')

    rospy.on_shutdown(saveData)

    # create empty matrices to save tracked data
    kinectTraj = np.empty(shape=[0,3])
    baxterTraj = np.empty(shape=[0,3])

    # Publish transform and marker
    while not rospy.is_shutdown():

        # base to refernce
        base_marker = lookupTransform(tf_listener, frame, '/base')
        trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))

        # marker to camera
        marker_camera = lookupTransform(tf_listener, '/ar_marker_'+str(markernum), '/kinect2_link')
        trans_camera, rot, rot_euler = getTfFromMatrix(np.linalg.inv(marker_camera))

        kinectTraj = np.vstack([kinectTraj,np.asarray(trans_camera)])
        baxterTraj = np.vstack([baxterTraj,np.asarray(trans_baxter)])
        rate.sleep()

if __name__=='__main__':
    main()
