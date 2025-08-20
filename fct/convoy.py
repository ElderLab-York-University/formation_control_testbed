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
from math import atan2, copysign, sin, cos, pi

import cv2 as cv
import numpy as np


RAD_TO_DEG = 180.0 / pi


def normalizeAngle(o):
    r'''Normalize a given angle to the range `[-pi, pi)`.
    '''
    return copysign(abs(o) % pi, o)


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

    def update(self, id, dt, target, obstacles, sensor_range):
        r'''Update the controller of the wheelchair by the given ID.
        '''
        controller = self.__convoy[id]
        controller.update(dt, target, obstacles, sensor_range)
        return controller.state


class Controller:
    r'''Base controller class.
    '''
    def __init__(self, target, settings):
        r'''Create a controller for a robot lined up behind the given target.
        '''
        separation = settings.get('separation', 1.0)
        (x, y, o) = target[:3]

        self.x = x - separation * cos(o)
        self.y = y - separation * sin(o)
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
        self.__separation = settings.get('separation', 1.0)

    def update(self, dt, target, obstacles, sensor_range):
        r'''Update the state of the controller from the state of the given target.
        '''
        self.iterate(dt)

        # Update the robot's linear and angular speeds to track its leader.

        (x, y) = target[:2]
        o = atan2(y - self.y, x - self.x)
        w = (o - self.o) / dt

        a = np.array([self.x, self.y])

        s = self.__separation
        b = np.array([x - s * cos(o), y - s * sin(o)])

        v = max(0.0, np.linalg.norm(b - a) / dt)

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

    def __attract(self, target, sensor_range):
        r'''Compute the attraction vector towards the given target.
        '''
        v = (target - np.array([self.x, self.y])) / sensor_range
        d = np.linalg.norm(v)

        return v * (self.__gain_attraction * d)

    def __repulse(self, obstacle, sensor_range):
        r'''Compute the attraction vector towards the given obstacle.
        '''
        v = (np.array([self.x, self.y]) - obstacle) / sensor_range
        d = np.linalg.norm(v)

        return v * (self.__gain_repulsion * ((sensor_range / d) - 1.0) / (d ** 3))

    def update(self, dt, target, obstacles, sensor_range):
        r'''Update the state of the controller from the state of the given target.
        '''
        self.iterate(dt)

        vector = self.__attract(np.array(target[:2]), sensor_range)

        for obstacle in obstacles:
            vector += self.__repulse(np.array(obstacle), sensor_range)

        vector *= sensor_range / (1.0 + len(obstacles))
        (dx, dy) = vector

        a = np.array([self.x, self.y])
        b = np.array([self.x + dx, self.y + dy])
        v = max(0.0, np.linalg.norm(b - a) / dt)

        o = atan2(dy, dx)
        w = (o - self.o) / dt

        self.v = v
        self.w = w


class ProximityMap:
    r'''Map of the environment where the robots are moving.
    '''
    def __init__(self, path, origin, resolution=0.1, obstacle_weight=2.0, obstacle_deviation=1.0):
        r'''Instantiate a map from the given file.
        '''
        obstacle_variance = 1.0 / (obstacle_deviation ** 2.0)

        occupancy_map = cv.imread(path, cv.IMREAD_GRAYSCALE)
        proximity_map = np.zeros(occupancy_map.shape, dtype=float)

        cells = dict()
        (m, n) = occupancy_map.shape
        for ((i, j), value) in zip(product(range(m), range(n)), occupancy_map.flat):
            if value == 1:
                cells[i, j] = (0.0, 0.0)
                proximity_map[i, j] = obstacle_weight

        while cells:
            neighbors = dict()
            for ((i, j), (x, y)) in cells.items():
                for i_s in range(-1, 2):
                    i_k = i + i_s
                    if i_k < 0 or i_k >= m:
                        continue

                    y_k = y + i_s * resolution

                    for j_s in range(-1, 2, 1 if i_s != 0 else 2):
                        j_k = j + j_s;
                        if j_k < 0 or j_k >= n:
                            continue

                        x_k = x + j_s * resolution
                        r_k = obstacle_weight / (1.0 + obstacle_variance * (x_k ** 2 + y_k ** 2))

                        r = proximity_map[i_k, j_k]
                        if r_k <= r:
                            continue

                        proximity_map[i_k, j_k] = r_k

                        neighbors[i_k, j_k] = (x_k, y_k)

            cells = neighbors

        self.__map = proximity_map
        self.origin = np.array(origin, dtype=float)
        self.resolution = resolution

    def __call__(self, x, y, o, obstacle_range):
        r'''Extract a section of the map of dimensions `(1 + 2 * obstacle_range, obstacle_range)`,
            aligned with the robot's position and orientation.
        '''
        (m, n) = self.__map.shape
        position = np.array([x, y], dtype=flloat)
        center = ((position - self.origin) / self.resolution).astype(int)
        center[0] = m - center[0] # Map coordinates grow from bottom-left, while matrix' grow from top-left

        rotation = cv.getRotationMatrix2D(center, -o * RAD_TO_DEG, 1.0)
        rotated = cv.warpAffine(self.__map, rotation, self.__map.shape)

        (j, i) = center
        shape = (1 + 2 * obstacle_range, obstacle_range)
        local_map = np.zeros(shape)

        rows_g = slice(max(i - obstacle_range, 0), min(i + obstacle_range + 1, m))
        cols_g = slice(j, min(j + obstacle_range + 1, n))

        rows_l = slice(
            0 if i >= obstacle_range else (obstacle_range - i),
            shape[0] - (i + obstacle_range + 1 - m if (i + obstacle_range) >= m else 0)
        )

        cols_l = slice(0, shape[1] - (j + obstacle_range + 1 - n if (j + obstacle_range) >= n else 0))

        local_map[rows_l, cols_l] = rotated[rows_g, cols_g]

        return local_map


class MRFC(Controller):
    r'''An MRFC agent.
    '''
    def __init__(self, target, settings):
        r'''Create a new MRFC agent.
        '''
        separation = settings.get('separation', 1.0)
        (x, y, o, v, w) = target
        super().__init__(
            x - separation * cos(o),
            y - separation * sin(o),
            o,
            v,
            w
        )

        self.__map = ProximityMap(
            settings['path'],
            settings['origin'],
            settings.get('resolution', 0.1),
            settings.get('obstacle_weight', 2.0),
            settings.get('obstacle_deviation', 1.0)
        )
