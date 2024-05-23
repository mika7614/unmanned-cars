#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Created on Sat Sep 11 17:21:55 2021

@author: Admin
"""

import numpy as np
import collections


KALMAN_MAX_LEN = 10
SIMPLE_MAX_LEN = 20
# all of these vectors must be normalized
# state = [cur_angle, safe_distance].T     state      ->x
# observe = [cur_angle, safe_distance].T   observe    ->z
# control = [steer_angle, speed].T
class KF:
    def __init__(self, x0):
        self.x = x0
        self.A = np.eye(x0.size)
        self.B = np.diag(np.array([0.0, 0.0, 0.0]))
        self.P = np.diag(np.array([388.1, 9226.4, 11901.8]))
        self.C = np.eye(x0.size)
        self.I = np.eye(x0.size)
        self.x_samples = collections.deque(maxlen=KALMAN_MAX_LEN)
        self.x_samples.append(x0)
        # self.w_samples = collections.deque(maxlen=KALMAN_MAX_LEN)
        # self.v_samples = collections.deque(maxlen=KALMAN_MAX_LEN)
        self.E = {}


    def iterate(self, u, z, Q, R):
        self.x_samples.append(self.x)
        # self.w_samples.append(w @ w.T)
        # self.v_samples.append(v @ v.T)
        # self.E['Q'] = np.mean(np.array(self.w_samples), axis=0)
        # self.E['R'] = np.mean(np.array(self.v_samples), axis=0)
        self.x = self.x_samples[-1]
        self.E['Q'] = Q
        self.E['R'] = R
        A = self.A
        B = self.B
        temp1 = np.dot(self.P, self.C.T)
        temp2 = np.dot(self.C, self.P)
        temp2 = np.dot(temp2, self.C.T)
        temp2 = np.linalg.inv(temp2 + self.E['R'])
        K = np.dot(temp1, temp2)
        # K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.E['R'])
        temp1 = z - np.dot(B, u) - np.dot(self.C, self.x)
        x = self.x + np.dot(K, temp1)
        # x = self.x + K @ (z - B @ u - self.C @ self.x)
        temp1 = self.I - np.dot(K, self.C)
        P = np.dot(temp1, self.P)
        # P = (self.I - K @ self.C) @ self.P
        x = np.dot(A, x) + np.dot(B, u)
        # x = A @ x + B @ u
        temp1 = np.dot(A, P)
        P = np.dot(temp1, A.T) + self.E['Q']
        # P = A @ P @ A.T + self.E['Q']
        self.x = x
        self.P = P
        return x

class simple_control:
    def __init__(self):
        self.size = SIMPLE_MAX_LEN
        self.hor_samples = collections.deque(maxlen=SIMPLE_MAX_LEN)
        self.coef = 60.0
        
    def iterate(self, hor):
        factor = np.var(self.hor_samples) * self.coef
        distance = np.abs(self.hor_samples[-1] - hor)
        if factor == 0:
            conf = 1
        else:
            conf = np.exp(-np.power(distance, 2) / factor)
        correct = conf * hor + (1 - conf) * self.hor_samples[-1]
        self.hor_samples.append(correct)
        return correct
        

if __name__ == "__main__":
    observe = np.load('observe.npy')
    endpoint = observe.shape[1]
    # endpoint = 800
    correct = np.zeros_like(observe[0, :endpoint])
    # ****************** simple controller ***************** #
    # sp_ctrl = simple_control()
    # for i in range(endpoint):
    #     correct[i] = sp_ctrl.iterate(observe[2, i])
    # ****************** kalman filter ********************* #
    displace = 4
    init_state = observe[:, 0]
    init_state = np.array([0, 0, 0])
    kf = KF(init_state)
    state = np.zeros_like(observe[:, :endpoint])
    state[:, 0] = init_state
    for i in range(1, endpoint):
        x_diff = np.sin(state[0, i-1])
        u = np.array([0, 0, x_diff])
        z = observe[:, i]
        Q = np.diag(np.array([1, 0.1, 1]))
        R = np.diag(np.abs(z - kf.x))
        state[:, i] = kf.iterate(u, z, Q, R)
