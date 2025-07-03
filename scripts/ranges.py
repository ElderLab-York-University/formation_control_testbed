#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2021 Helio Perroni Filho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

r'''Data structure for storing and manipulating integer ranges.

    To run automated tests, open a terminal prompt and enter:

        $ python ranges.py

    This module has been successfully tested in versions 2.7.17 and 3.6.9 of the
    Python interpreter.

    Author: Helio Perroni Filho
'''


from bisect import bisect_left, bisect_right


class Ranges(object):
    r'''Data structure for storing and manipulating integer ranges.

        A "range" is defined by a starting point (inclusive) and an ending point
        (exclusive), for example the range `(1, 4)` contains the values 1, 2, and 3.

        **Implementation Notes**

        Ranges are stored internally as a flat list, e.g. the ranges
        `[(1, 3), (5, 7), (8, 9)]` are stored as `[1, 3, 5, 7, 8, 9]`. This is a very
        compact representation: space complexity is `O(n)` in the number of ranges,
        with a low constant factor. Since Python lists are implemented as dynamic arrays,
        this is also very convenient for modern memory architectures, enabling effective
        use of burst access.

        Inside a flat list, even indexes are defined to lie between ranges, while odd
        indexes lie inside. In the above example, index `0` would lie before range `(1, 3)`,
        index `2` between that and `(5, 7)`, and index `5` inside the range `(8, 9)`.

        This arrangement makes possible the use of binary tree search (implemented in the
        `bisect` package) to query ranges in `O(log n)` time. For example, to query which
        (if any) range includes a given value `x`, we can call `i = bisect(flat_list, x)`
        to find the index of the value closest to `x`. Then we check `i`'s parity: if `i`
        is odd, `x` lies inside the range `(flat_list[i-1], flat_list[i])`, otherwise it's
        not inside any existing range.

        The downside of using flat lists to represent range collections is that range
        insertion and deletion is of time complexity `O(n)` in the average case, with
        a better (amortized) performance of `O(1)` only for insertion / deletion at the
        end.

        However, performance tests indicate this is unlikely to be a problem in practice.
        This is because, as ranges are added to a collection, inter-range spaces are gradually
        filled; eventually ranges start merging, putting a cap on the number of individual
        ranges. Experiments combining random range additions and deletions also failed to
        produce significant time performance issues.

        Should scenarios with problematic time performance be identified in the future,
        there is still the option of replacing the flat list implementation with one
        based on [Discrete Interval Encoding Trees](http://web.engr.oregonstate.edu/~erwig/diet/).
        This would likely require the addition of tree-balancing logic (as in red-black
        trees) to ensure optimal search times.

        However, in the absence of obvious time performance drawbacks, the current implementation
        remains preferable for its low memory complexity, short query times, relative simplicity
        and ease of maintenance.
    '''
    def __init__(self, ranges=None):
        r'''Create a new range collection.
        '''
        self.__ranges = []

        if ranges is None:
            return

        for (start, end) in ranges:
            self.add(start, end)

    def __replace(self, i, j, inserting):
        r'''Replaces the ranges between `i` and `j` with the given values.

            If `i == j` nothing is removed.
        '''
        # List insertion anywhere except at the end takes O(n) time. For the price of
        # insert()'ing each element separately (and therefore shifting all following
        # values as many times) we might as well build a new list object.
        ranges = self.__ranges
        if i >= len(ranges):
            self.__ranges.extend(inserting)
        elif j == 0:
            self.__ranges = inserting
            self.__ranges.extend(ranges)
        else:
            self.__ranges = ranges[:i] + inserting + ranges[j:]

    def __find(self, start, end, exclusive=True):
        r'''Returns the indexes corresponding to the start and end values of the ranges that
            overlap with the range `(start, end)`, as well as the start and end limit values.

            If `exclusive` is `False` (default is `True`), the returned range will include
            ranges that are _contiguous_ with the query range but not overlapping.

            For example, for state `[(1, 3), (5, 7), (8, 9)]`:

            1. `self.__find(2, 8, True)` will cover ranges `[(1, 3), (5, 7)]`
            2. `self.__find(2, 8, False)` will cover ranges `[(1, 3), (5, 7), (8, 9)]`

            This is used to find ranges for replacement in `add()`.

            If the range is not valid (i.e. `start >= end`) returns the special value
            `(None, None, None, None)`.
        '''
        if start >= end:
            # Trivial range, empty by definition.
            return (None, None, None, None)

        # bisect_left / bisect_right perform binary tree search, time complexity is O(log n).
        # If the collection is empty then i == j == 0.
        if exclusive:
            i = bisect_right(self.__ranges, start)
            j = bisect_left(self.__ranges, end)
        else:
            i = bisect_left(self.__ranges, start)
            j = bisect_right(self.__ranges, end)

        if i % 2 != 0:
            # The beginning of the range is enclosed in an existing range.
            i -= 1
            start = self.__ranges[i]

        if j % 2 != 0:
            # The end of the range is enclosed in an existing range.
            end = self.__ranges[j]
            j += 1

        return (i, j, start, end)

    def add(self, start, end):
        r'''Adds a new range into the data structure from `start` to `end`.

            If the range already exists, or is contained within an already existing range,
            then nothing is done. If the new range partially overlaps existing ranges,
            they are merged into a new range that covers the union of the new and old
            ranges.

            Examples:

            1. Original state `[(1, 2)]`, add `(3, 5)`, new state: `[(1, 2), (3, 5)]`
            2. Original state `[(1, 6)]`, add `(3, 5)`, new state: `[(1, 6)]`
            3. Original state `[(1, 4)]`, add `(3, 5)`, new state: `[(1, 5)]`

            Time complexity of this method is `O(1)` for insertion at the end of the
            collection, and `O(n)` everywhere else.
        '''
        (i, j, start, end) = self.__find(start, end, False)
        if i is None:
            # Invalid range, do nothing.
            return

        # Remove overlapping / enclosed ranges and insert the new range.
        self.__replace(i, j, [start, end])

    def delete(self, start, end):
        r'''Deletes the range from `start` to `end` from the data structure.

            If the range does not exist, does nothing. If the range partially overlaps an
            existing range, the existing range is truncated. If the range completely
            overlaps an existing range, then the existing range is be deleted. Examples:

            1. Original state `[(1, 6)]`, delete `(-3, -1)`, new state: `[(1, 6)]`
            2. Original state `[(1, 6)]`, delete `(-1, 10)`, new state: `[]`
            3. Original state `[(1, 6)]`, delete `(4, 10)`, new state: `[(1, 4)]`

            Time complexity of this method is `O(1)` for deletion from the end of the
            collection, and `O(n)` everywhere else.
        '''
        (i, j, found_start, found_end) = self.__find(start, end)
        if i is None:
            # Invalid range, do nothing.
            return

        if i == j:
            # The delete range lies outside any existing range.
            return

        truncated = []
        if found_start < start:
            # If the start of the delete range intersects with an existing range,
            # reinsert a truncated version of that range in the collection.
            truncated = [found_start, start]

        if end < found_end:
            # If the end of the delete range intersects with an existing range,
            # reinsert a truncated version of that range in the collection.
            truncated.extend([end, found_end])

        if truncated:
            self.__replace(i, j, truncated)
        else:
            del self.__ranges[i:j]

    def get(self, start=None, end=None):
        r'''Returns a list of ranges contained in the data that intersect with `start` and `end`.

            If either `start` or `end` is `None`, the minimal / maximal existing range value is
            used.

            Examples:

            1. State `[(1, 3), (5, 7)]`: `ranges.get(4, 5) == []`
            2. State `[(1, 3), (5, 6)]`: `ranges.get(4, 6) == [(5, 6)]`
            3. State `[(1, 3), (5, 6)]`: `ranges.get(2, 9) == [(1, 3), (5, 6)]`
            4. State `[(1, 3), (5, 6)]`: `ranges.get() == [(1, 3), (5, 6)]`

            Time complexity of this method is `O(log n)` due to the use of binary tree
            search for locating matching ranges.
        '''
        ranges = self.__ranges
        if not ranges:
            return []

        if start is None:
            start = self.__ranges[0]

        if end is None:
            end = self.__ranges[-1]

        (i, j, _, _) = self.__find(start, end)
        if i is None:
            # Invalid range, result is empty by definition.
            return []

        return [(a, b) for (a, b) in zip(ranges[i:j:2], ranges[i+1:j+1:2])]
