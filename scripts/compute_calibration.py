#!/usr/bin/env python

# compute_calibration.py: Code to compute absolute orientation from collected points
# Author: Nishanth Koganti
# Date: 2016/06/16
# Source: http://math.stackexchange.com/questions/745234/calculate-rotation-translation-matrix-to-match-measurement-points-to-nominal-poi

# import modules
import tf
import rospy
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# matplotlib settings
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

axes = {'titlesize'  : '15',
        'labelsize'  : '12',
        'titleweight': 'bold',
        'labelweight': 'bold'}

matplotlib.rc('font', **font)
matplotlib.rc('axes', **axes)

def absOrientation(x,y):
    # number of samples
    nSamples = x.shape[0]

    # center data
    xMean = x.mean(axis=0)
    yMean = y.mean(axis=0)

    xTemp = x - xMean
    yTemp = y - yMean

    # get the variance
    xSD = np.mean(np.sum(xTemp**2, 1))
    ySD = np.mean(np.sum(yTemp**2, 1))

    # get covariance matrix
    covarMatrix = np.dot(yTemp.T,xTemp) / nSamples

    # apply singular value decomposition
    U,D,V = np.linalg.svd(covarMatrix,full_matrices=True,compute_uv=True)
    V=V.T.copy()

    S = np.diag(np.asarray([1,1,np.sign(np.linalg.det(V)*np.linalg.det(U))]))

    # get scaling factor
    c = np.trace(np.dot(np.diag(D), S)) / xSD

    # get rotation matrix
    R = c*np.dot(np.dot(U,S), V.T)

    # get translation vector
    t = yMean - np.dot(R,xMean)

    # compute transformation error
    xOut = (np.dot(R,x.T)).T + t
    errs = np.sqrt(np.sum((y - xOut)**2,axis=1))
    err = errs.sum()/nSamples

    # output results
    print 'Calibration Error: ',err

    return xOut,R,t


def main():
    # initialize ros node
    rospy.init_node('compute_calibration')

    # get path to folder containing recorded data
    filesPath = rospy.get_param('~files_path')

    # load trajectories
    kinectTraj = np.loadtxt('%skinect_trajectory' % (filesPath), delimiter=',')
    baxterTraj = np.loadtxt('%sbaxter_trajectory' % (filesPath), delimiter=',')

    # compute absolute orientation
    kinectOut, rot, trans = absOrientation(kinectTraj,baxterTraj)

    # plot both point sets together
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # plot the data
    Axes3D.scatter(ax, kinectOut[:,0], kinectOut[:,1], kinectOut[:,2], s=20, c='r')
    Axes3D.scatter(ax, baxterTraj[:,0],baxterTraj[:,1], baxterTraj[:,2], s=20, c='b')

    # add labels and title
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.title('Kinect-Baxter Calibration')
    plt.legend(['Kinect','Baxter'])
    plt.show()

    # get rotation matrix as quaternion and euler angles
    euler = tf.transformations.euler_from_matrix(rot)
    quat = tf.transformations.quaternion_from_euler(*euler)

    # save results to yaml file
    f = open('%skinect_calibration.yaml' % (filesPath), 'w')
    lines = ['trans: [', 'rot: [', 'rot_euler: [']

    for elem in trans: lines[0] += str(elem) + ', '
    lines[0] += ']\n'
    for elem in quat: lines[1] += str(elem) + ', '
    lines[1] += ']\n'
    for elem in euler: lines[2] += str(elem) + ', '
    lines[2] += ']\n'

    lines.append('parent: /base\n')
    lines.append('child: /kinect2_link\n')

    f.writelines(lines)
    f.close()

if __name__=='__main__':
    main()
