;; Software License Agreement (BSD License)
;;
;;  Copyright (c) 2025 Helio Perroni Filho & Elder Lab
;;  All rights reserved.
;;
;;  Redistribution and use in source and binary forms, with or without
;;  modification, are permitted provided that the following conditions
;;  are met:
;;
;;   * Redistributions of source code must retain the above copyright
;;     notice, this list of conditions and the following disclaimer.
;;   * Redistributions in binary form must reproduce the above
;;     copyright notice, this list of conditions and the following
;;     disclaimer in the documentation and/or other materials provided
;;     with the distribution.
;;   * Neither the name of the Elder Lab nor the names of its
;;     contributors may be used to endorse or promote products derived
;;     from this software without specific prior written permission.
;;
;;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;;  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;;  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
;;  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
;;  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;;  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;  POSSIBILITY OF SUCH DAMAGE.

extensions [
  csv
  py
]

globals [
  ;; ID of the guide in the current track file.
  guide-id

  ;; Simulation clock, stores the current global timestamp.
  clock

  ;; Initial simulation time.
  t_0

  ;; Time passed since the start of the simulation in seconds.
  t_passed

  ;; Path to the map image.
  path-map

  ;; Map resolution in meters.
  map-resolution

  ;; Position of the map's bottom-left pixel relative to
  ;; the origin of the map coordinate frame, in meters.
  map-x0
  map-y0

  ;; Number of per-guide obstacle / wheelchair collisions.
  obstacle-collisions

  ;; Number of per-guide pedestrian / wheelchair collisions.
  pedestrian-collisions

  ;; Total collisions across all guides.
  all-collisions
]

breed [pedestrians pedestrian]

pedestrians-own [
  ;; Pedestrian's track ID.
  id

  ;; Pedestrian position in the map coordinate frame, in meters.
  x-coordinate
  y-coordinate
]

breed [guides guide]

guides-own [
  ;; Timestamp of the last guide state update.
  timestamp

  ;; Guide's track ID.
  id

  ;; Number of collisions reported during this guide's trip.
  collisions

  ;; Guide position in the map coordinate frame, in meters.
  x-coordinate
  y-coordinate

  ;; Guide orientation relative to the map coordinate frame, in radians.
  ;; Follows robotics conventions: values increase counter-clockwise, the
  ;; range is between [-pi, pi), and the origin points Right.
  orientation

  ;; Guide linear speed in meters per second.
  linear-speed

  ;; Guide angular speed in radians per second.
  angular-speed
]

breed [wheelchairs wheelchair]

wheelchairs-own [
  ;; ID of the convoy to which this wheelchair belongs.
  convoy-id

  ;; Wheelchair position in the map coordinate frame, in meters.
  x-coordinate
  y-coordinate

  ;; Wheelchair orientation relative to the map coordinate frame, in radians.
  ;; Follows robotics conventions: values increase counter-clockwise, the
  ;; range is between [-pi, pi), and the origin points Right.
  orientation

  ;; Wheelchair linear speed in meters per second.
  linear-speed

  ;; Wheelchair angular speed in radians per second.
  angular-speed
]

to reset-settings
  set settings (ifelse-value
    method = "Naive" [
      "separation: 1.0"
    ]
    method = "APF" [
      (word
        "separation: 1.0\n"
        "attraction_gain: 0.1\n"
        "repulsion_gain: 0.2"
      )
    ]
    [ ;; method = MRFC
      (word
        "separation: 1.0\n"
        "path: \"" path-map "\"\n"
        "origin: [" map-x0 ", " map-y0 "]\n"
        "resolution: " map-resolution "\n"
        "obstacle_weight: 2.0\n"
        "obstacle_deviation: 1.0"
      )
    ]
  )
end

;; Initialize the simulation.
to setup
  py:setup py:python3
  file-close-all
  clear-all

  ;; TODO: Load map parameters from file.
  set path-map "data/map.png"
  set map-resolution 0.1
  set map-x0 -60.0
  set map-y0 -40.0

  py:set "path_map" path-map
  py:set "origin" (list map-x0 map-y0)
  py:set "resolution" map-resolution

  py:set "folder" "data/tracks"
  py:set "method" method
  py:set "reach" (list min-reach max-reach)

  (py:run
    "from fct import datasets"
    "from fct.convoy import Convoy"
    "datasets.update(folder, method, reach)"
    "convoy = Convoy()"
  )

  ifelse all-tracks? [
    py:run "datasets.enqueue(folder)"
  ] [
    ;; Ask the user to select a track file, aborting the operation if they refuse.
    let path-dataset user-file
    if path-dataset = false [
      stop
    ]

    py:set "path_dataset" path-dataset
    py:run "datasets.open(path_dataset)"
  ]

  ;; Import the occupancy map.
  import-pcolors path-map

  set clock false
  set all-collisions 0

  reset-ticks
end

;; Run the simulation.
to go
  carefully [
    iterate
  ] [
    print error-message
    stop
  ]
end

;; Perform one iteration step in the simulation.
to iterate
  let row py:runresult "next(datasets)"
  if row = nobody [
    ;; Record the guide's collision count.
    ask guides [
      py:set "record" (list id collisions)
      py:run "datasets.recordCollisions(*record)"
    ]

    ;; Set up the environment for the next track.
    set clock false

    ask turtles [
      die
    ]

    ;; Delete all wheelchair controllers.
    py:run "convoy.clear()"

    stop
  ]

  ;; Reset the environment for a new guide.
  let now (item 0 row)
  ifelse clock = false [
    set t_0 now
    set t_passed 0.0
    set obstacle-collisions 0
    set pedestrian-collisions 0
    clear-all-plots
    clear-drawing
  ] [
    set t_passed (clock - t_0)
  ]

  ;; Update the simulation clock, taking notice if the time changed.
  let ticked? (clock != now)
  set clock now

  ;; Iterate a guide or pedestrian depending on the nature of the current track record.
  let guide? (item 6 row)
  ifelse guide? [
    iterate-guide row
  ] [
    iterate-pedestrian row
  ]

  ;; Remove the pedestrian if we reached the end of its track.
  let track-id (item 1 row)
  let last? (item 7 row)
  if last? [
    ask pedestrians with [id = track-id] [
      die
    ]
  ]

  ;; Update the tick count only if the simulation clock moved.
  if ticked? [
    tick
  ]
end

;; Iterate the simulated guide.
to iterate-guide [row]
  set guide-id (item 1 row)
  let convoy-guide (one-of guides with [id = guide-id])
  ifelse convoy-guide = nobody [
    create-guides 1 [
      set convoy-guide self
      set id guide-id
      set color red
      set size 10

      update-guide row
      pen-down
    ]

    create-convoy convoy-guide
  ] [
    let dt clock - [timestamp] of convoy-guide
    ask convoy-guide [
      update-guide row
    ]

    update-convoy dt convoy-guide
  ]
end

;; Set the state of a guide.
to update-guide [row]
  let track-x (item 2 row)
  let track-y (item 3 row)
  let track-v (item 4 row)
  let track-o (item 5 row)

  if timestamp != 0 [
    set angular-speed (track-o - orientation) / (clock - timestamp)
  ]

  set timestamp clock
  set x-coordinate track-x
  set y-coordinate track-y
  set orientation track-o
  set linear-speed track-v

  setxy (to-world-x track-x) (to-world-y track-y)
  set heading to-heading track-o
end

;; Update the state of the simulated wheelchair convoy.
to update-convoy [dt convoy-guide]
  ;; If there is no convoy associated to this guide, create it.
  if not any? [out-link-neighbors] of convoy-guide [
    create-convoy convoy-guide
  ]

  ;; If a collision is detected before the update...
  if collision? convoy-guide [
    ;; Delete then convoy.
    ask wheelchairs with [convoy-id = [who] of convoy-guide] [
      die
    ]

    ;; Delete all wheelchair controllers.
    py:run "convoy.clear()"

    stop
  ]

  ;; Otherwise, update wheelchair poses.
  let target convoy-guide
  loop [
    ;; This assumes that any guide / wheelchair has at most one follower.
    let follower (one-of [out-link-neighbors] of target)
    if follower = nobody [
      update-collision-count convoy-guide
      stop
    ]

    let obstacles []

    ask [other turtles in-radius (sensor-range / map-resolution)] of follower [
      set obstacles (fput (list x-coordinate y-coordinate) obstacles)
    ]

    ask [patches in-radius (sensor-range / map-resolution) with [pcolor = 0]] of follower [
      set obstacles (fput (to-map pxcor pycor) obstacles)
    ]

    py:set "id" [who] of follower
    py:set "dt" dt
    py:set "target" (get-state target)
    py:set "obstacles" obstacles
    py:set "sensor_range" sensor-range

    set-state follower (py:runresult "convoy.update(id, dt, target, obstacles, sensor_range)")

    ;; Update the target for the next loop iteration.
    set target follower
  ]
end

;; Create a convoy of wheelchairs behind a guide.
to create-convoy [convoy-guide]
  let target convoy-guide

  create-wheelchairs wheelchair-count [
    set convoy-id [who] of convoy-guide

    set color green
    set size 10

    create-link-from target [
      set color black
      set thickness 2
    ]

    ;; Initialize the wheelchair's controller and its state in the simulation.
    py:set "id" who
    py:set "method" method
    py:set "target" (get-state target)
    py:set "settings" settings
    set-state self (py:runresult "convoy.add(id, method, target, settings)")

    ;; Set this wheelchair as the target for the next one.
    set target self
  ]
end

;; Report if any of the convoy wheelchairs have collided with an obstacle or pedestrian.
to-report collision? [convoy-guide]
  let target convoy-guide

  loop [
    ;; This assumes that any guide / wheelchair has at most one follower.
    let follower (one-of [out-link-neighbors] of target)
    if follower = nobody [
      report false
    ]

    let collision-found? false
    ask follower [
      set collision-found? (
        (any? patches in-radius (collision-threshold / map-resolution) with [pcolor = 0]) or
        (any? other turtles in-radius (collision-threshold / map-resolution))
      )

      ;; Set this wheelchair as the search key for the next one.
      set target self
    ]

    if collision-found? [
      report true
    ]
  ]
end

;; Update collision counts for the given guide's convoy.
to update-collision-count [convoy-guide]
  ask wheelchairs with [convoy-id = [who] of convoy-guide] [
    if any? patches in-radius (collision-threshold / map-resolution) with [pcolor = 0] [
      set all-collisions (all-collisions + 1)
      set obstacle-collisions (obstacle-collisions + 1)
      ask convoy-guide [
        set collisions (collisions + 1)
      ]
    ]

    if any? other turtles in-radius (collision-threshold / map-resolution) [
      set all-collisions (all-collisions + 1)
      set pedestrian-collisions (pedestrian-collisions + 1)
      ask convoy-guide [
        set collisions (collisions + 1)
      ]
    ]
  ]
end

;; Iterate a simulated pedestrian.
to iterate-pedestrian [row]
  let track-id (item 1 row)
  let track-x (item 2 row)
  let track-y (item 3 row)
  let track-o (item 5 row)

  ;; Add the pedestrian to the simulation if they're not already included.
  let iterating (one-of pedestrians with [id = track-id])
  if iterating = nobody [
    create-pedestrians 1 [
      set iterating self
      set id track-id
      set color blue
      set size 10
    ]
  ]

  ask iterating [
    set x-coordinate track-x
    set y-coordinate track-y

    setxy (to-world-x track-x) (to-world-y track-y)
    set heading to-heading track-o
  ]
end

;; Return the state of the given agent.
to-report get-state [agent]
  report (list
    [x-coordinate] of agent
    [y-coordinate] of agent
    [orientation] of agent
    [linear-speed] of agent
    [angular-speed] of agent
  )
end

;; Set the state of the given agent
to set-state [agent state]
  ask agent [
    set x-coordinate (item 0 state)
    set y-coordinate (item 1 state)
    set orientation (item 2 state)
    set linear-speed (item 3 state)
    set angular-speed (item 4 state)

    ;; Update the simulation state.
    carefully [
      setxy (to-world-x x-coordinate) (to-world-y y-coordinate)
      set heading (to-heading orientation)
    ] [
      ;; Nothing to do.
    ]
  ]
end

;; Convert the given world coordinates to the map reference frame.
to-report to-map [world-x world-y]
  report (list
    (to-map-coordinate world-x map-x0)
    (to-map-coordinate world-y map-y0)
  )
end

;; Convert the given world coordinate to the map frame of reference.
to-report to-map-coordinate [world-k map-k0]
  report world-k * map-resolution + map-k0
end

;; Convert the given map X coordinate to the NetLogo world frame of reference.
to-report to-world-x [map-x]
  report to-world-coordinate map-x map-x0 (world-width - 1)
end

;; Convert the given map Y coordinate to the NetLogo world frame of reference.
to-report to-world-y [map-y]
  report to-world-coordinate map-y map-y0 (world-height - 1)
end

;; Convert the given map coordinate to the NetLogo world frame of reference.
to-report to-world-coordinate [map-k map-k0 world-last]
  let world-k (map-k - map-k0) / map-resolution
  set world-k max (list world-k 0)
  set world-k min (list world-k world-last)
  report world-k
end

;; Convert an orientation from radians in map reference frame to degrees in NetLogo world frame.
to-report to-heading [direction]
  let angle -180.0 * direction / pi
  ifelse angle < -90.0 [
    report 450.0 + angle
  ] [
    report 90.0 + angle
  ]
end
@#$#@#$#@
GRAPHICS-WINDOW
0
10
1409
620
-1
-1
1.0
1
10
1
1
1
0
0
0
1
0
1400
0
600
0
0
1
ticks
30.0

BUTTON
755
30
826
66
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
830
30
896
66
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
905
30
1232
150
Pedestrians (per guide)
Time (s)
Count
0.0
60.0
0.0
10.0
true
false
"" ""
PENS
"default" 60.0 0 -16777216 true "" "plotxy t_passed count pedestrians\nif t_passed >= 60.0 [\n  ; Scroll the range of the plot so only the\n  ; last 1 hour worth of points is visible.\n  set-plot-x-range precision (t_passed - 60.0) 2 precision t_passed 2\n]"

SLIDER
10
120
195
153
wheelchair-count
wheelchair-count
1
4
3.0
1
1
NIL
HORIZONTAL

SLIDER
10
280
195
313
collision-threshold
collision-threshold
0.1
1.0
0.3
0.1
1
m
HORIZONTAL

PLOT
906
156
1232
322
Collisions (per guide)
Time (s)
Collisions
0.0
60.0
0.0
10.0
true
false
"" ""
PENS
"obstacles" 60.0 0 -13345367 true "" "plotxy t_passed obstacle-collisions\nif t_passed >= 60.0 [\n  ; Scroll the range of the plot so only the\n  ; last 1 minute worth of points is visible.\n  set-plot-x-range precision (t_passed - 60.0) 2 precision t_passed 2\n]"
"pedestrians" 60.0 0 -2674135 true "" "plotxy t_passed pedestrian-collisions\nif t_passed >= 60.0 [\n  ; Scroll the range of the plot so only the\n  ; last 1 minute worth of points is visible.\n  set-plot-x-range precision (t_passed - 60.0) 2 precision t_passed 2\n]"
"total" 1.0 0 -16777216 true "" "plotxy t_passed (obstacle-collisions + pedestrian-collisions)\nif t_passed >= 60.0 [\n  ; Scroll the range of the plot so only the\n  ; last 1 minute worth of points is visible.\n  set-plot-x-range precision (t_passed - 60.0) 2 precision t_passed 2\n]"

SLIDER
10
240
195
273
sensor-range
sensor-range
0.1
2.0
1.0
0.1
1
m
HORIZONTAL

CHOOSER
10
70
195
115
method
method
"Naive" "APF" "MRFC"
0

SWITCH
10
30
137
63
all-tracks?
all-tracks?
0
1
-1000

MONITOR
755
75
895
120
Guide ID
guide-id
17
1
11

MONITOR
717
129
897
174
Total Collisions (all guides)
all-collisions
17
1
11

SLIDER
10
160
195
193
min-reach
min-reach
0.0
100.0
70.0
1.0
1
m
HORIZONTAL

SLIDER
10
200
195
233
max-reach
max-reach
0.0
100.0
100.0
1.0
1
m
HORIZONTAL

INPUTBOX
10
320
277
431
settings
separation: 1.0
1
1
String

BUTTON
172
320
227
353
Reset
reset-settings
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

@#$#@#$#@
## WHAT IS IT?

[AirChair](https://elderlab-york-university.github.io/airchair/) is a semi-autonomous human transportation system composed of multiple wheelchairs operating as a convoy. The first wheelchair follows an on-foot human guide, the second wheelchair follows the first, and so on. Each wheelchair independently tracks its target and performs motion planning to follow along while steering clear of obstacles.

The AirChair motion planner, dubbed Mobile Robot Following Controller (MRFC), computes linear and angular speeds based on:

* The wheelchair leader's distance and linear speed;
* The presence and location of stationary and moving obstacles.

This is an instance of _formation control,_ a popular subject in mobile robotics. Therefore the question arises as to how MRFC compares to other solutions found in the academic literature. Ideally this would be evaluated quantitatively, through a simulation that would abstract away other components involved in the actual process of following a leader.

The Formation Control Testbed (FCT) is a NetLogo model for testing and quantitative comparison of formation control algorithms. The model features the following characteristics:

* A flat spatial environment, divided in cells of either free or occupied space;
* A number of simulated pedestrians moving across this environment, one of which is the convoy's guide;
* One or more autonomous wheelchairs, each following its leader.

The identity and location of each wheelchair's leader, the occupancy status of environment cells, and the locations of other pedestrians are all given by the model, which is also responsible for executing the speed commands received from motion planner instances.

## HOW IT WORKS

Agents in the FTC model come in three categories:

* Pedestrians (blue arrows): these are people moving in random trajectories across the environment, who may get in the way of the convoy.
* The Guide (red arrow): this is a simulated person taking the role of the guide of a wheelchair convoy;
* Wheelchairs (green arrows): one or more autonomous wheelchairs following a human guide as a convoy;

The main difference between the Guide and other pedestrians is that the former is expected to move in a relatively consistent manner from a point A to a point B, while the latter could have all sorts of behaviors, including staying in place.

Pedestrian and guide motion is simulated by replaying person tracks generated from the [ATC pedestrian tracking dataset](https://dil.atr.jp/crest2010_HRI/ATC_dataset/). Each track is a sequence of timestamped and ID'd position records representing the motion of a single person through the ATC shopping center. When a track is selected as a guide, all (sections of) the tracks spanning the same time period are also loaded to play the role of pedestrians. Tracks are stored in [SQLite](https://www.sqlite.org/) database files to allow for fast seeking through the data.

Tracks are ranked by _reach,_ defined for a track `t = [p_0, p_1, ..., p_n]` as `max(distance(p_0, p_k) for p_k in t)`. This is a better measure for selecting guide tracks than duration in time, or even the sum of offsets between locations, since those values can be high for tracks where the person just wobbles around the same location for a long time.

Wheelchair motion is determined by the application of a formation control algorithm. At present the FCT model provides two options:

* A `naive` leader-following algorithm with no obstacle avoidance capability;
* An `APF` algorithm based on Artificial Potential Fields.

Additional algorithms, including the MRFC algorithm that initially motivated the creation of the FCT model, will follow in the near future.

## HOW TO USE IT

On the top-left corner of the interface there are widgets for controlling basic agent features:

* `all-tracks?` indicates whether all SQLite database files will be played in sequence or a single one selected for playing --- in the latter case a file opening dialog will be shown for selecting the file;
* `method` specifies the leader-following algorithm used by the wheelchairs;
* `wheelchair-count` determines the number of simulated wheelchairs in the convoy.
* `min-reach` specifies the shortest a track can be in reach to be selected as a guide;
* `max-reach` specifies the longest a track can be in reach to be selected as a guide.

On the bottom-left corner there are widgets to configure the `APF` leader-following algorithm (these are all ignored when the `naive` algorithm is selected):

* `sensor-range` determines the farthest distance at which pedestrians and occupied cells are still considered for avoidance;
* `collision-threshold` is the distance under which an obstacle is considered to have collided with a wheelchair;
* `attraction-gain` controls how strongly each wheelchair is pulled towards its leader;
* `repulsion-gain` controls how strongly each wheelchair is pushed away from obstacles.

Finally, on the top-right corner there are the `Setup` and `Go` buttons to control the execution of the model, along with some visualization widgets:

* `Guide ID` displays the ID of the current guide --- this is useful to know which file to replay in case of a specially tricky tracks that requires its own tuning;
* `Total Collisions (all guides)` shows the total number of collisions across all guide tracks in the current session. This helps give a general overview of the performance of different algorithms;
* `Pedestrians (per guide)` and `Collisions (per guide)` respectively plot the number of pedestrians present and the accumulated number of collisions as a function of time. In contrast to the `Total Collisions` monitor, these reset each time a new guide track is loaded. Collisions are plotted by total number (black) as well as collisions with pedestrians (red) and static obstacles (blue).

## THINGS TO NOTICE

As the guide moves about the environment, its trajectory is plotted in the environment. This helps identifying difficult spots in the route.

Links are used to bind each wheelchair to its leader. This makes it easier to identify the position of each wheelchair in the convoy.

Wheelchairs are initially placed in a straight line behind the guide, one meter apart from each other. If a collision is detected at this point the wheelchairs are deleted and re-instantiated at the next iteration, after the guide has moved to its next position. This is repeated until the wheelchairs can be placed in their initial positions without collision. Additionally, if a collision is detected while the wheelchairs are moving, the convoy is also deleted and re-instantiated as above. The point of this procedure is to avoid spamming the collision count when wheelchairs get stuck into walls or chased by pedestrians.

During execution, per-guide collision totals are written to a database file `collisions-<method>-<timestamp>.db`. These files are re-created before each session.

## THINGS TO TRY

How does the number of wheelchairs affect the collision count of either algorithm?

Try both the `naive` and `APF` algorithms and notice the difference in the number of collisions, especially against pedestrians.

With the `APF` method selected, try varying the algorithm parameters (bottom left widgets), especially the attraction and repulsion gains. How do these changes affect the algorithm's effectiveness? You may want to select one of the trickier tracks for this, instead of running through them all every time.

## EXTENDING THE MODEL

The current leader-follower algorithms both operate directly on wheelchair pose (position + heading). This assumes the wheelchairs can immediately accelerate / decelerate to reach any destination. A more realistic approach would be to keep track of linear and angular speeds, computing position changes from them while enforcing acceleration limits.

The `APF` algorithm considers obstacles all around the wheelchair when computing repulsion vectors. However, since the wheelchairs are supposed to only ever move forward, it would make sense to only consider obstacles in the front, while also enforcing a limitation against backward motion.

Instead of one convoy at a time, multiple convoys could be simultaneously simulated by selecting more than one guide. Most of the logic for this is already in place, the main challenge is extracing the right data from the ATC dataset. Alternatively, instead of relying on recorded tracks, pedestrian motion could also be simulated through the use of an APF-based algorithm.

## NETLOGO FEATURES

The model uses anonymous functions to dynamically bind the leader-following method during model setup. This is more efficient than checking the value of the `method` widget at every iteration, but has the side effect that the method cannot be changed while the model is running. In practice this shouldn't be much of a problem, since for testing purposes it makes little sense to change algorithms in the middle of a session.

Access to SQLite database files is implemented through the Python [sqlite3](https://docs.python.org/3/library/sqlite3.html) package and integrated to the FTC model through the [Py extension](https://ccl.northwestern.edu/netlogo/docs/py.html). Both the extension and a Python runtime must be installed for the model to work.

## RELATED MODELS

* Ant Lines
* Flocking
* Moths

## CREDITS AND REFERENCES

See the model [repository](https://github.com/ElderLab-York-University/formation_control_testbed) for future developments.

## COPYRIGHT AND LICENSE

_Copyright &copy; 2025 Helio Perroni Filho & Elder Lab_
_All rights reserved._

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the Elder Lab nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

person-rotatable
true
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.4.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
1
@#$#@#$#@
