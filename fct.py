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

r'''Python API for the Formation Control Testbed (FTC) NetLogo model.

    Author: Helio Perroni Filho
'''


import sqlite3
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


@dataclass
class Track:
    r'''Basic information about a person track.
    '''
    id: int
    t_0: float
    t_n: float

    @property
    def period(self):
        r'''Return the period covered by this track.
        '''
        return (self.t_0, self.t_n)


class Collisions:
    r'''Collect collision counts from experiments.
    '''
    def __init__(self, folder, method):
        r'''Create a new collisions record database.
        '''
        if not isinstance(folder, Path):
            folder = Path(folder)

        timestamp = datetime.now().strftime('%Y-%m-%d-%H%M%S')
        path = folder / f'collisions-{method}-{timestamp}.db'

        self.database = sqlite3.connect(path)
        self.cursor = self.database.cursor()
        self.cursor.execute('CREATE Table CollisionCount(dataset, guide, count)')

    def close(self):
        r'''Close this collision log.
        '''
        self.database.close()

    def insert(self, dataset, guide, count):
        r'''Insert a collision count record to the log.
        '''
        self.cursor.execute('INSERT INTO CollisionCount VALUES(?, ?, ?)', (dataset, guide, count))
        self.database.commit()


class Dataset:
    r'''Connection to a tracking dataset.
    '''
    def __init__(self, path, min_reach, max_reach):
        r'''Open a tracking dataset file.
        '''
        if not isinstance(path, Path):
            path = Path(path)

        self.database = sqlite3.connect(path)
        self.name = path.stem[4:]

        self.guides = self.database.cursor()
        self.guides.execute('SELECT id, t0, tn FROM Track WHERE reach BETWEEN ? AND ? ORDER BY reach DESC', (min_reach, max_reach))

        self.readings = self.database.cursor()

        self.guide = None
        self.last_record = None

    def close(self):
        r'''Close the access to this dataset.
        '''
        self.database.close()

    def __next__(self):
        r'''Return the next dataset reading.
        '''
        if self.guide is None:
            # Will raise a StopIteration if all guides have been iterated through.
            self.guide = Track(*next(self.guides))
            period = self.guide.period

            self.last_record = set(
                self.readings.execute('SELECT MAX(timestamp), id FROM Reading WHERE timestamp BETWEEN ? AND ? GROUP BY id', period)
            )

            self.readings.execute('SELECT * FROM Reading WHERE timestamp BETWEEN ? AND ? ORDER BY timestamp', period)

        reading = next(self.readings, None)
        if reading is None:
            self.guide = None
            return None # Signify the end of the current guide's session.

        is_guide = (reading[1] == self.guide.id)
        is_last = (reading[:2] in self.last_record)

        return reading + (is_guide, is_last)


class Datasets:
    r'''Access to person tracking datasets.
    '''
    def __init__(self):
        r'''Create a new tracking dataset client.
        '''
        self.collisions = None
        self.dataset = None
        self.queue = list()

        self.reach = (0.0, 100.0)

    def __next__(self):
        r'''Return the next pedestrian record.
        '''
        dataset = self.dataset
        if dataset is None:
            if self.queue:
                self.open(self.queue.pop(0))
            else:
                self.collisions.close()
                raise StopIteration()

        try:
            return next(self.dataset)
        except StopIteration:
            self.dataset.close()
            self.dataset = None
            return None

    def enqueue(self, path):
        r'''Enqueue the datasets matching the given search path.
        '''
        self.queue.extend(Path(path).glob('atc-*.db'))

    def open(self, path):
        r'''Open the given database.
        '''
        dataset = self.dataset
        if dataset is not None:
            dataset.close()

        self.dataset = Dataset(path, *self.reach)

    def recordCollisions(self, guide, count):
        r'''Record a collisions count for this guide.
        '''
        self.collisions.insert(self.dataset.name, guide, count)

    def update(self, folder, method, reach):
        r'''Update the dataset access object for another session.
        '''
        self.reach = reach
        if self.collisions is not None:
            self.collisions.close()

        self.collisions = Collisions(folder, method)


datasets = Datasets()
