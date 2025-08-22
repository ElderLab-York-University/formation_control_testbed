#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (C) 2025 Elder Lab & authors
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Elder Lab nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

r'''Controllers and support artifacts used in simulation.

    Author: Helio Perroni Filho
'''


import yaml
from itertools import product
from math import atan2, copysign, inf, isfinite, sin, cos, pi
from traceback import format_exc

import cv2 as cv
import nlopt as nl
import numpy as np


def distance(a, b):
    r'''Return the Euclidean distance between two vectors.
    '''
    return np.linalg.norm(a - b)


def normalizeAngle(o):
    r'''Normalize a given angle to the range `[-pi, pi)`.
    '''
    o = (o + pi) % (2 * pi)
    if o < 0.0:
        o += 2.0 * pi

    return o - pi


def toLocalFrame(pose, positions):
    r'''Convert the given positions to the local frame of the given pose.
    '''
    (x, y, o) = pose
    shift = np.array([x, y])
    rotation = np.array([ # Rotate points clockwise.
        [cos(o), -sin(o)],
        [sin(o), cos(o)]
    ])

    return (positions - shift) @ rotation


def truncate(x, a, b):
    r'''Truncate the value `x` to the range `[a, b]`.
    '''
    return max(a, min(x, b))


class Convoy:
    r'''Logic for controlling a convoy.
    '''
    def __init__(self):
        r'''Create a new convoy.
        '''
        self.__convoy = dict()

    def add(self, id, method, target, settings):
        r'''Add a controller for the wheelchair by the given ID.
        '''
        settings = yaml.load(settings, yaml.FullLoader)
        controller = eval(method)(target, settings)
        self.__convoy[id] = controller

        return controller.state

    def clear(self):
        r'''Delete all wheelchair controllers.
        '''
        self.__convoy.clear()

    def update(self, id, dt, target, obstacles):
        r'''Update the controller of the wheelchair by the given ID.
        '''
        try:
            controller = self.__convoy[id]
            controller.update(dt, target, obstacles)
            return controller.state
        except Exception as e:
            print(format_exc())
            raise e


class Controller:
    r'''Base controller class.
    '''
    def __init__(self, target, settings):
        r'''Create a controller for a robot lined up behind the given target.
        '''
        self.sensor_range = settings.get('sensor_range', 1.0)
        self.separation = settings.get('separation', 1.0)

        (x, y, o) = target[:3]

        self.x = x - self.separation * cos(o)
        self.y = y - self.separation * sin(o)
        self.o = o
        self.v = 0.0
        self.w = 0.0

    def iterate(self, dt):
        r'''Update the robot's pose based on current orientation and linear / angular speeds.
        '''
        o = self.o
        v = self.v
        w = self.w

        cos_o = cos(o)
        sin_o = sin(o)
        wdt = w * dt

        if abs(w) > 1e-4:
            r = v / w

            cos_wdt = cos(wdt)
            sin_wdt = sin(wdt)

            self.x += r * (cos_wdt * sin_o + cos_o * sin_wdt - sin_o)
            self.y += r * (sin_wdt * sin_o - cos_o * cos_wdt + cos_o)
            self.o = normalizeAngle(o + wdt)
        else:
            s = v * dt
            self.x += s * cos_o
            self.y += s * sin_o

    @property
    def pose(self):
        r'''Return the robot pose as a tuple.
        '''
        return (
            self.x,
            self.y,
            self.o
        )

    @property
    def state(self):
        r'''Return the controller state as a tuple.
        '''
        return (
            self.x,
            self.y,
            self.o,
            self.v,
            self.w
        )


class Naive(Controller):
    r'''A naive controller with no obstacle avoidance capabilities.
    '''
    def __init__(self, target, settings):
        r'''Create a new naive controller trailing the given target.
        '''
        super().__init__(target, settings)

    def update(self, dt, target, obstacles):
        r'''Update the state of the controller from the state of the given target.
        '''
        self.iterate(dt)

        # Update the robot's linear and angular speeds to track its leader.

        (x, y) = target[:2]
        o = atan2(y - self.y, x - self.x)
        w = (o - self.o) / dt

        a = np.array([self.x, self.y])

        s = self.separation
        b = np.array([x - s * cos(o), y - s * sin(o)])

        v = distance(a, b) / dt

        self.v = v
        self.w = w


class APF(Controller):
    r'''Controller based on Artificial Potential Fields.
    '''
    def __init__(self, target, settings):
        r'''Create a new naive controller trailing the given target.
        '''
        super().__init__(target, settings)
        self.__gain_attraction = settings.get('attraction_gain', 0.1)
        self.__gain_repulsion = settings.get('repulsion_gain', 0.2)

    def __attract(self, target):
        r'''Compute the attraction vector towards the given target.
        '''
        v = (target - np.array([self.x, self.y])) / self.sensor_range
        d = np.linalg.norm(v)

        return v * (self.__gain_attraction * d)

    def __repulse(self, obstacle):
        r'''Compute the attraction vector towards the given obstacle.
        '''
        v = (np.array([self.x, self.y]) - obstacle) / self.sensor_range
        d = np.linalg.norm(v)

        return v * (self.__gain_repulsion * ((self.sensor_range / d) - 1.0) / (d ** 3))

    def update(self, dt, target, obstacles):
        r'''Update the state of the controller from the state of the given target.
        '''
        self.iterate(dt)

        vector = self.__attract(np.array(target[:2]))

        for obstacle in obstacles:
            vector += self.__repulse(np.array(obstacle))

        vector *= self.sensor_range / (1.0 + len(obstacles))
        (dx, dy) = vector

        a = np.array([self.x, self.y])
        b = np.array([self.x + dx, self.y + dy])
        v = distance(a, b) / dt

        o = atan2(dy, dx)
        w = (o - self.o) / dt

        self.v = v
        self.w = w


class ProximityMap:
    r'''Map of the environment where the robots are moving.
    '''
    def __init__(self, settings):
        r'''Instantiate a proximity map.
        '''
        self.resolution = settings.get('resolution', 0.1)
        self.obstacle_weight = settings.get('obstacle_weight', 2.0)
        self.obstacle_deviation = settings.get('obstacle_deviation', 1.0)
        self.sensor_range = settings.get('sensor_range', 1.0)
        self.target_radius = settings.get('target_radius', 1.0)

        self.__obstacle_variance = 1.0 / (self.obstacle_deviation ** 2.0)
        self.__reach = round(self.sensor_range / self.resolution)

        self.__map = None

    def update(self, pose, target_position, obstacles):
        r'''Update the proximity map.
        '''
        reach = self.__reach
        m = 1 + 2 * reach
        n = 1 + reach

        proximity_map = np.zeros((m, n), dtype=float)

        if not obstacles:
            return

        obstacles = np.array(obstacles)
        obstacle_cells = np.round(toLocalFrame(pose, obstacles) / self.resolution).astype(int)
        obstacle_weight = self.obstacle_weight

        cells = dict()
        for (obstacle, (j, i)) in zip(obstacles, obstacle_cells):
            if distance(target_position, obstacle) <= self.target_radius:
                continue

            i += reach
            if (0 <= i < m) and (0 <= j < n):
                proximity_map[i, j] = obstacle_weight
                cells[i, j] = (0.0, 0.0)

        while cells:
            neighbors = dict()
            for ((i, j), (x, y)) in cells.items():
                for i_s in range(-1, 2):
                    i_k = i + i_s
                    if i_k < 0 or i_k >= m:
                        continue

                    y_k = y + i_s * self.resolution

                    for j_s in range(-1, 2, 1 if i_s != 0 else 2):
                        j_k = j + j_s;
                        if j_k < 0 or j_k >= n:
                            continue

                        x_k = x + j_s * self.resolution
                        r_k = obstacle_weight / (1.0 + self.__obstacle_variance * (x_k ** 2 + y_k ** 2))

                        r = proximity_map[i_k, j_k]
                        if r_k <= r:
                            continue

                        proximity_map[i_k, j_k] = r_k

                        neighbors[i_k, j_k] = (x_k, y_k)

            cells = neighbors

        self.__map = proximity_map

    def __call__(self, x, y):
        r'''Return the obstacle proximity value for the given coordinates in the local reference frame.
        '''
        if self.__map is None:
            return 0.0

        (m, n) = self.__map.shape

        i = self.__reach + round(y / self.resolution)
        if not (0 <= i < m):
            return 0.0

        j = round(x / self.resolution)
        if not (0 <= j < n):
            return 0.0

        return self.__map[i, j]


def curve(x, curvature):
    r'''Compute the value of the `y` coordinate for the given curve at the `x` coordinate.
    '''
    # A curvature of (effectively) zero implies a straight line.
    if abs(curvature) < 1e-6:
        return 0.0

    r = abs(1.0 / curvature)

    r2 = r ** 2.0
    x2 = x ** 2.0
    if r2 < x2:
        return inf

    return copysign(r - (r2 - x2) ** 0.5, curvature)


class MRFC(Controller):
    r'''An MRFC agent.
    '''
    def __init__(self, target, settings):
        r'''Create a new MRFC agent.
        '''
        super().__init__(target, settings)

        self.__distance_gain = settings.get('distance_gain', 0.8)
        self.__target_closest = self.separation
        self.__linear_max_speed = settings.get('linear_max_speed', 3.0)
        self.__angular_max_speed = settings.get('angular_max_speed', 3.0)

        target_distant = settings.get('target_distant', 1.5)
        self.__linear_gain = (target_distant - self.__target_closest) / self.__linear_max_speed
        self.__linear_bias = self.__target_closest

        self.__track = list()
        self.__track_local = list()
        self.__map = ProximityMap(settings)

        obstacle_deviation = self.__map.obstacle_deviation
        obstacle_weight = self.__map.obstacle_weight
        obstacle_threshold = settings.get('obstacle_threshold', 0.5)
        self.__obstacle_threshold = obstacle_weight / (1.0 + (obstacle_threshold / obstacle_deviation) ** 2.0)

        self.__opt = nl.opt(nl.LN_NELDERMEAD, 1)
        self.__opt.set_lower_bounds(-1.0 / self.__map.resolution)
        self.__opt.set_upper_bounds( 1.0 / self.__map.resolution)
        self.__opt.set_maxtime(1.0 / settings.get('frequency', 20.0))
        self.__opt.set_min_objective(self.__cost)
        self.__opt.set_stopval(1e-6)

        self.__curvature = 0.0

    def __cost(self, parameters, gradients):
        r'''Curvature optimization cost function.
        '''
        total_cost = 0.0

        curvature = parameters[0]
        for position in self.__track_local:
            x = position[0]
            y = curve(x, curvature)
            if isfinite(y):
                total_cost += self.__map(x, y) + (y - position[1]) ** 2.0
            else: # Heavily penalize any curve that can't be drawn up to the target.
                total_cost += self.sensor_range ** 2.0 + self.__map.obstacle_weight

        return total_cost

    def update(self, dt, target, obstacles):
        r'''Update the state of the controller from the state of the given target and obstacles.
        '''
        self.iterate(dt)

        # Add latest position to track as adequate.
        target_position = np.array(target[:2])
        if len(self.__track) == 0 or distance(target_position, self.__track[-1]) > self.__map.resolution:
            self.__track.append(target_position)

        self.__track_local.clear()

        track = self.__track
        self.__track = list()

        pose = self.pose
        position = np.array(pose[:2])

        for position_track in track:
            position_local = toLocalFrame(pose, position_track)
            if position_local[0] > 0:
                self.__track.append(position_track)
                self.__track_local.append(position_local)

        self.__map.update(pose, target_position, obstacles)

        if distance(position, target_position) <= self.__target_closest:
            self.v = 0.0
            self.w = 0.0
            return

        try:
            optimized = self.__opt.optimize(np.array([self.__curvature]))
            self.__curvature = optimized[0]
        except Exception as e:
            self.v = 0.0
            self.w = 0.0
            print(e)
            return

        # Check if the route is free of obstacles.
        for x in np.arange(0.0, self.__track_local[-1][0], self.__map.resolution):
            y = curve(x, self.__curvature)
            if not isfinite(y):
                break

            if self.__map(x, y) > self.__obstacle_threshold:
                self.v = 0.0
                self.w = 0.0
                return

        k = self.__distance_gain
        max_w = self.__angular_max_speed

        v_target = target[3]
        d_target = distance(position, target_position)

        d_follow = self.__linear_bias + self.__linear_gain * v_target
        v_follow = truncate(v_target + k * (d_target - d_follow), 0.0, self.__linear_max_speed)
        w_follow = v_follow * self.__curvature

        if abs(w_follow) > max_w:
            w_follow = copysign(max_w, w_follow)
            v_follow = w_follow / self.__curvature

        self.v = v_follow
        self.w = w_follow
