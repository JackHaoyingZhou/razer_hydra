# Author: Adnan Munawar
# Acknowledgement: https://jekel.me/2015/Least-Squares-Sphere-Fit/
# amunawar@wpi.edu

import rospy
from razer_hydra.msg import Hydra
from geometry_msgs.msg import Transform
import time
import numpy as np
import math
from matplotlib import rcParams
#   3D plot of the
import matplotlib.pyplot as plt
import PyKDL
from tf_conversions import posemath
from mpl_toolkits.mplot3d import Axes3D

hydra_msg = Hydra()
new_data = False


class CalibData():
    def __init__(self):
        self._sample_size = 4
        self.pos_arr = np.zeros([self._sample_size, 3])
        self.pivot_arr = np.zeros([100, 3])
        self._index_ctr = 0
        self._pvt_index_ctr = 0
        self._calibrated = False
        self._scale = 0.3
        self._r = 0
        self._cx = 0
        self._cy = 0
        self._cz = 0

        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        self.pINh = PyKDL.Vector(0.1, 0.0, 0.0)
        # point w.r.t hydra frame
        self.T_pINh = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), self.pINh)
        # point w.r.t origin frame
        self.T_pINo = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.0))
        # hydra w.r.t origin frame
        self.T_hINo = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.0))
        # rotation pivot w.r.t origin frame
        self.T_rINo = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.0))
        # rotation pivot w.r.t hydra device
        self.T_rINh = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.0))
        # estimated rotation pivot w.r.t origin frame
        self.T_eINo = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.0, 0.0, 0.0))

        self.pivot_arr[0,0] = self.T_pINh.p[0]
        self.pivot_arr[0,1] = self.T_pINh.p[1]
        self.pivot_arr[0,1] = self.T_pINh.p[2]
        self._pvt_index_ctr = self._pvt_index_ctr + 1

    def check_validity(self, Ta):
        if Ta.p.Norm() < 0.5:
            return True
        else:
            return False

    def clear_data(self):
        self._index_ctr = 0
        pass

    def add_data(self, p):
        print 'Data Point: {}\r'.format(calData.get_index_counter())
        self.T_pINo = self.toFrame(p.transform)
        self.T_hINo = self.T_pINo * self.T_pINh.Inverse()
        self.pos_arr[self._index_ctr, 0] = self.T_pINo.p[0]
        self.pos_arr[self._index_ctr, 1] = self.T_pINo.p[1]
        self.pos_arr[self._index_ctr, 2] = self.T_pINo.p[2]
        self._index_ctr = self._index_ctr + 1
        if self._index_ctr % self._sample_size == 0:
            self.clear_data()
            [r, cx, cy, cz] = self.sphere_fit()
            self.T_eINo.p = PyKDL.Vector(cx, cy, cz)
            self.T_eINo.M = self.T_hINo.M
            T_eINh = self.T_hINo.Inverse() * self.T_eINo
            if self.check_validity(T_eINh) is True:
                n = float(self._pvt_index_ctr)
                self.T_rINh.p = ((n / (n+1)) * self.T_rINh.p) + ((1 / (n+1)) * T_eINh.p)
                #print 'Estimated Pivot in Hydra :\n{}'.format(T_eINh.p)
                print '*** Iterated Pivot in Hydra :\n\t{}'.format(self.T_rINh.p)
                self.pivot_arr[int(n), 0] = T_eINh.p[0]
                self.pivot_arr[int(n), 1] = T_eINh.p[1]
                self.pivot_arr[int(n), 2] = T_eINh.p[2]
                self._pvt_index_ctr = self._pvt_index_ctr + 1
                # if self._pvt_index_ctr % 2 == 0:
                self.plot_radii()
            else:
                print 'Estimated Pivot Too Far :\n{}'.format(T_eINh.p)

    def get_index_counter(self):
        return self._index_ctr

    def is_calibrated(self):
        return self._calibrated

    def get_sample_size(self):
        return self._sample_size

    def plot_radii(self):
        self.ax.clear()
        eH = self.pivot_arr
        self.ax.scatter(eH[:, 0], eH[:, 1], eH[:, 2], zdir='z', s=20, marker='o',
                        c='r', rasterized=True)
        rH = self.T_rINh.p
        self.ax.scatter(rH[0], rH[1], rH[2], zdir='z', marker='^', s=80, c='g',
                        rasterized=True)
        pH = self.pINh
        self.ax.scatter(pH[0], pH[1], pH[2], zdir='z', s=80, c='black',
                        rasterized=True)
        self.ax.set_aspect('equal')
        [x_min, x_max] = self.get_bounds(self.pivot_arr[0:self._pvt_index_ctr, 0])
        [y_min, y_max] = self.get_bounds(self.pivot_arr[0:self._pvt_index_ctr, 1])
        [z_min, z_max] = self.get_bounds(self.pivot_arr[0:self._pvt_index_ctr, 2])

        self.ax.set_xlim3d(x_max, x_min)
        self.ax.set_ylim3d(y_max, y_min)
        self.ax.set_zlim3d(z_max, z_min)
        self.ax.set_xlabel('$x$ (m)', fontsize=16)
        self.ax.set_ylabel('\n$y$ (m)', fontsize=16)
        self.ax.axis('equal')
        plt.draw()
        plt.pause(0.001)

    # Code from https://jekel.me/2015/Least-Squares-Sphere-Fit/
    # fit a sphere to X,Y, and Z data points returns the radius and center points of the best fit sphere
    def sphere_fit(self):
        #   Assemble the A matrix
        spX = self.pos_arr[:, 0]
        spY = self.pos_arr[:, 1]
        spZ = self.pos_arr[:, 2]

        spX = np.array(spX)
        spY = np.array(spY)
        spZ = np.array(spZ)
        A = np.zeros((len(spX), 4))
        A[:, 0] = spX * 2
        A[:, 1] = spY * 2
        A[:, 2] = spZ * 2
        A[:, 3] = 1

        #   Assemble the f matrix
        f = np.zeros((len(spX), 1))
        f[:, 0] = (spX * spX) + (spY * spY) + (spZ * spZ)
        C, residules, rank, singval = np.linalg.lstsq(A, f)

        #   solve for the radius
        t = (C[0] * C[0]) + (C[1] * C[1]) + (C[2] * C[2]) + C[3]
        radius = math.sqrt(t)
        self._calibrated = True
        self._r = radius
        self._cx = C[0]
        self._cy = C[1]
        self._cz = C[2]

        return radius, C[0], C[1], C[2]

    def get_bounds(self, arr):
        max = np.amax(arr)
        min = np.amin(arr)
        return [min, max]

    def toFrame(self, p):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(p.rotation.x,
                                         p.rotation.y,
                                         p.rotation.z,
                                         p.rotation.w),
                           PyKDL.Vector(p.translation.x, p.translation.y, p.translation.z))


trigger = False
rising_edge = False


def hydra_cb(data):
    global hydra_msg, new_data, trigger, rising_edge
    if data.paddles[1].buttons[0] is True and trigger is False:
        rising_edge = True
        trigger = True

    if data.paddles[1].buttons[0] is False:
        trigger = False
        rising_edge = False

    if rising_edge is True:
        rising_edge = False
        hydra_msg = data
        new_data = True


rospy.init_node('hydra_pivot_calibration')
sub = rospy.Subscriber('/hydra_calib', Hydra, hydra_cb, queue_size=10)

calData = CalibData()
rate = rospy.Rate(100)
cnt = 3
while cnt > 0:
    print 'Starting Calibration in {}...'.format(cnt)
    cnt = cnt - 1
    time.sleep(1)
_calib = False
while not rospy.is_shutdown():
    if new_data and calData.get_index_counter() < calData.get_sample_size():
        calData.add_data(hydra_msg.paddles[1])
        new_data = False
    time.sleep(0.01)

sub.unregister()
