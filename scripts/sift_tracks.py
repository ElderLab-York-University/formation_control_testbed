from ranges import Ranges

import csv
from dataclasses import dataclass
from sys import argv

import numpy as np
from numpy.linalg import norm


@dataclass
class Track:
    id: int
    t_0: float
    t_n: float
    a: np.ndarray
    b: np.ndarray
    reach: float
    is_guide: bool = False


def sift_tracks(dataset):
    tracks = dict()

    with open(f'data/tracks/atc-{dataset}.csv', newline='') as csv_file:
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

    ranges = Ranges()
    for track in sorted(tracks.values(), key=lambda x: -x.reach):
        t_0 = int(track.t_0 * 1000.0)
        t_n = int(track.t_n * 1000.0)
        if not ranges.get(t_0, t_n):
            track.is_guide = True
            ranges.add(t_0, t_n)

    with open(f'data/tracks/atc-{dataset}-tracks.csv', 'w', newline='') as csv_file:
        max_reach = 0
        mean_reach = 0
        writer = csv.writer(csv_file)
        for track in sorted(tracks.values(), key=lambda x: x.t_0):
            writer.writerow([track.id, track.t_0, track.t_n, f'{track.reach:.2f}', track.is_guide])
            max_reach = max(max_reach, track.reach)
            mean_reach += track.reach

        mean_reach = float(mean_reach) / len(tracks)
        print(f'Maximum reach: {max_reach}')
        print(f'Mean reach: {mean_reach}')

datasets = [
    '20121024',
    '20121028',
    '20121031',
    '20121104',
    '20121107',
    '20121111',
    '20121114',
    '20121118',
    '20121121',
    '20121125'
]

for dataset in datasets:
    print(f'Sifting tracks in dataset {dataset}...')
    sift_tracks(dataset)
