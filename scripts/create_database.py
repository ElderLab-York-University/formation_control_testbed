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

r'''Script to generate SQLite databases for the FCT model from ATC dataset CSV files.

    Author: Helio Perroni Filho
'''


import csv
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.linalg import norm


@dataclass
class Track:
    r'''Basic information about a person track.
    '''
    id: int
    t_0: float
    t_n: float
    a: np.ndarray
    b: np.ndarray
    reach: float


def create_database(path):
    r'''Create a SQLite database from the CSV file at the given path.
    '''
    tracks = dict()

    database = sqlite3.connect(f'data/tracks/{path.stem}.db')
    cursor = database.cursor()

    cursor.execute('CREATE TABLE Reading(timestamp, id, x, y, speed, heading)')
    cursor.execute('CREATE INDEX reading_timestamp ON Reading(timestamp)')
    cursor.execute('CREATE INDEX reading_id ON Reading(id)')

    with open(path, newline='') as csv_file:
        for row in csv.reader(csv_file):
            id = int(row[1])
            if id < 0:
                continue

            t = float(row[0])
            x = float(row[2]) * 0.001
            y = float(row[3]) * 0.001
            p = np.array([x, y])

            track = tracks.get(id)
            if track is None:
                tracks[id] = Track(id, t, t, p, p, 0.0)
            else:
                track.t_n = t
                a = track.a
                max_d = track.reach
                d = norm(a - p)
                if d > max_d:
                    track.b = p
                    track.reach = d

            record = (
                t, # timestamp
                id, # id
                x, # x
                y, # y
                float(row[5]) * 0.001, # speed
                float(row[6]) # heading
            )

            cursor.execute('INSERT INTO Reading VALUES(?, ?, ?, ?, ?, ?)', record)

    cursor.execute('CREATE TABLE Track(id, t0, tn, reach)')
    cursor.execute('CREATE UNIQUE INDEX track_id ON Track(id)')
    cursor.execute('CREATE INDEX track_t0 ON Track(t0)')
    cursor.execute('CREATE INDEX track_tn ON Track(tn)')
    cursor.execute('CREATE INDEX track_reach ON Track(reach)')

    for track in tracks.values():
        record = (track.id, track.t_0, track.t_n, track.reach)
        cursor.execute('INSERT INTO Track VALUES(?, ?, ?, ?)', record)

    database.commit()

    database.close()


def main():
    r'''Main entry point.
    '''
    from sys import argv
    create_database(Path(argv[1]))


if __name__ == '__main__':
    main()
