# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import itertools
import numpy as np
from scipy.stats.distributions import chi2
from typing import List

# add project directory to python path to enable relative imports
import os
import sys

PACKAGE_PARENT = ".."
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params
from student.trackmanagement import Track
from student.measurements import Measurement
from student.filter import Filter


class Association:
    """Data association class with single nearest neighbor association and gating based on Mahalanobis distance"""

    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []

    def associate(self, track_list: List[Track], meas_list: List[Measurement], KF: Filter):
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############

        N = len(track_list)
        M = len(meas_list)

        self.unassigned_meas = list(range(M))
        self.unassigned_tracks = list(range(N))
        self.association_matrix = np.asmatrix(np.inf * np.ones((N, M)))

        for i, j in itertools.product(range(N), range(M)):
            if self.gating(dist := self.MHD(track_list[i], meas_list[j], KF), meas_list[j].sensor):
                self.association_matrix[i, j] = dist

        ############
        # END student code
        ############

    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        # the following only works for at most one track and one measurement
        track_idx, meas_idx = np.unravel_index(np.argmin(self.association_matrix), self.association_matrix.shape)
        if (not np.isfinite(self.association_matrix[track_idx, meas_idx])) or len(self.unassigned_tracks) == 0:
            return np.nan, np.nan

        self.association_matrix = np.delete(self.association_matrix, track_idx, axis=0)
        self.association_matrix = np.delete(self.association_matrix, meas_idx, axis=1)

        update_track = self.unassigned_tracks[track_idx]
        update_meas = self.unassigned_meas[meas_idx]

        # remove from lists
        self.unassigned_tracks.remove(update_track)
        self.unassigned_meas.remove(update_meas)

        ############
        # END student code
        ############
        return update_track, update_meas

    def gating(self, MHD, sensor):
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############

        return MHD < chi2.ppf(params.gating_threshold, sensor.dim_meas)

        ############
        # END student code
        ############

    def MHD(self, track: Track, meas: Measurement, KF: Filter):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############

        H = meas.sensor.get_H(track.x)
        S = KF.S(track, meas, H)
        gamma = KF.gamma(track, meas)

        return (gamma.T * np.linalg.inv(S) * gamma)[0, 0]

        ############
        # END student code
        ############

    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)

        # update associated tracks with measurements
        while self.association_matrix.shape[0] > 0 and self.association_matrix.shape[1] > 0:
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print("---no more associations---")
                break
            track = manager.track_list[ind_track]

            # check visibility, only update tracks in fov
            if not meas_list[0].sensor.in_fov(track.x):
                continue

            # Kalman update
            print("update track", track.id, "with", meas_list[ind_meas].sensor.name, "measurement", ind_meas)
            KF.update(track, meas_list[ind_meas])

            # update score and track state
            manager.handle_updated_track(track)

            # save updated track
            manager.track_list[ind_track] = track

        # run track management
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)

        for track in manager.track_list:
            print("track", track.id, "score =", track.score)
