# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

PACKAGE_PARENT = ".."
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params
from student.trackmanagement import Track
from student.measurements import Measurement


class Filter:
    """Kalman filter class"""

    def __init__(self):
        pass

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############

        return np.matrix(
            [
                [1.0, 0.0, 0.0, params.dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, params.dt, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, params.dt],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ]
        )

        ############
        # END student code
        ############

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############

        f = lambda p: (1 / p) * np.power(params.dt, p) * params.q

        return np.matrix(
            [
                [f(3), 0.0, 0.0, f(2), 0.0, 0.0],
                [0.0, f(3), 0.0, 0.0, f(2), 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [f(2), 0.0, 0.0, f(1), 0.0, 0.0],
                [0.0, f(2), 0.0, 0.0, f(1), 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )

        ############
        # END student code
        ############

    def predict(self, track: Track):
        ############
        # Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        x_plus: np.matrix = track.x
        P_plus: np.matrix = track.P

        x_min = self.F() * x_plus
        P_min = self.F() * P_plus * self.F().T + self.Q()

        track.set_x(x_min)
        track.set_P(P_min)

        ############
        # END student code
        ############

    def update(self, track: Track, meas: Measurement):
        ############
        # Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############

        gamma = self.gamma(track, meas)
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P * H.T * np.linalg.inv(S)
        x_plus = track.x + K * gamma
        P_plus = (np.identity(6) - K * H) * track.P

        track.set_x(x_plus)
        track.set_P(P_plus)

        ############
        # END student code
        ############
        track.update_attributes(meas)

    def gamma(self, track: Track, meas: Measurement):
        ############
        # Step 1: calculate and return residual gamma
        ############

        return meas.z - meas.sensor.get_hx(track.x)

        ############
        # END student code
        ############

    def S(self, track: Track, meas: Measurement, H: np.matrix) -> np.matrix:
        ############
        # Step 1: calculate and return covariance of residual S
        ############

        return H * track.P * H.T + meas.R

        ############
        # END student code
        ############
