extensions [
  csv
]

globals [
  ;; ID of the dataset from which the current track was taken.
  dataset-id

  ;; ID of the guide in the current track file.
  guide-id

  ;; Path to the output collisions report CSV file.
  path-collisions

  ;; Path to the current track's input dataset CSV file.
  path-track

  ;; List of pedestrian tracks.
  tracks

  ;; Simulation clock, stores the current global timestamp.
  clock

  ;; Initial simulation time.
  t_0

  ;; Time passed since the start of the simulation in seconds.
  t_passed

  ;; Map resolution in meters.
  map-resolution

  ;; Position of the map's bottom-left pixel relative to
  ;; the origin of the map coordinate frame, in meters.
  map-x0
  map-y0

  ;; Number of reported obstacle / wheelchair collisions.
  obstacle-collisions

  ;; Number of reported pedestrian / wheelchair collisions.
  pedestrian-collisions

  ;; Anonymous function used to update convoy wheelchairs.
  update-convoy-method
]

breed [pedestrians pedestrian]

pedestrians-own [
  ;; Pedestrian's track ID.
  id
]

breed [guides guide]

guides-own [
  ;; Timestamp of the last guide state update.
  t

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
  ;; Timestamp of the last state update.
  t

  ;; The human guide or wheelchair followed by this wheelchair.
  target

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

;; Initialize the simulation.
to setup
  clear-all
  file-close-all

  set path-track false
  set tracks []

  ifelse all-tracks? [
    file-open "data/tracks/all-tracks.csv"
    while [not file-at-end?] [
      let row csv:from-row file-read-line
      set tracks (lput row tracks)
    ]

    update-track
  ] [
    update-path-track
  ]

  ;; Interrupt the setup if the initial track path wasn't set.
  if path-track = false [
    stop
  ]

  set path-collisions (word "data/collisions-" method ".csv")
  carefully [
    file-delete path-collisions
  ] [
    ;; Nothing to do.
  ]

  set clock false

  ;; TODO: Load map parameters from file.
  set map-resolution 0.1
  set map-x0 -60.0
  set map-y0 -40.0

  ;; Import the occupancy map.
  import-pcolors "data/map.gif"

  set update-convoy-method (ifelse-value
    method = "naive"
      [[[convoy-guide] -> update-convoy-naive convoy-guide]]
    ; method = "APF"
      [[[convoy-guide] -> update-convoy-apf convoy-guide]]
  )

  set obstacle-collisions 0
  set pedestrian-collisions 0

  reset-ticks
end

;; Run the simulation.
to go
  file-open path-track
  if file-at-end? [
    file-close-all
    stop
  ]

  iterate
end

;; Perform one iteration step in the simulation.
to iterate
  let row csv:from-row file-read-line

  ;; Update the simulation clock, taking notice if the time changed.
  let now (item 0 row)
  ifelse clock = false [
    set t_0 now
    set t_passed 0.0
  ] [
    set t_passed (clock - t_0)
  ]

  let ticked? (clock != now)
  set clock now

  ;; Iterate a guide or pedestrian depending on the nature of the current track record.
  let track-id (item 1 row)
  ifelse track-id = guide-id [
    iterate-guide row
  ] [
    iterate-pedestrian row
  ]

  ;; Remove the pedestrian if we reached the end of its track.
  let track-last? ((item 6 row) = "Y")
  if track-last? [
    ask pedestrians with [id = track-id] [
      die
    ]
  ]

  ;; Update the tick count only if the simulation clock moved.
  if ticked? [
    tick
  ]

  ;; If we reached the end of the current track file...
  if file-at-end? [
    ;; Record the guide's collision count.
    ask guides [
      file-open path-collisions
      file-print (word dataset-id "," id "," collisions)
    ]

    ;; Stop here if no tracks left to play.
    if (length tracks) = 0 [
      stop
    ]

    ;; Load the next track.
    update-track

    ;; Set up the environment for the next track.
    set clock false
    set obstacle-collisions 0
    set pedestrian-collisions 0
    clear-all-plots
    clear-drawing

    ask turtles [
      die
    ]
  ]
end

;; Load a track path from the filesystem.
to update-path-track
  ;; Ask the user to select a track file, aborting the operation if they refuse.
  set path-track user-file
  if path-track = false [
    stop
  ]

  ;; Parse the track path to extract the dataset ID and guide ID.
  let track-name path-track
  let separator "/"
  loop [
    let index (position separator track-name)
    ifelse index = false [
      let index-ext (position "." track-name)
      set dataset-id (substring track-name 4 12)
      set guide-id (read-from-string (substring track-name 13 index-ext))
      stop
    ] [
      let index-start (index + 1)
      let index-end (length track-name)
      set track-name (substring track-name index-start index-end)
    ]
  ]
end

;; Update the current track.
to update-track
  if (length tracks) = 0 [
    stop
  ]

  let track (item 0 tracks)
  set tracks (remove-item 0 tracks)

  set dataset-id (item 0 track)
  set guide-id (item 1 track)

  set path-track (word "data/tracks/atc-" dataset-id "-" guide-id ".csv")
end

;; Iterate the simulated guide.
to iterate-guide [row]
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
  ] [
    ask convoy-guide [
      update-guide row
    ]
  ]

  update-convoy convoy-guide
end

;; Set the state of a guide.
to update-guide [row]
  let track-x (item 2 row)
  let track-y (item 3 row)
  let track-v (item 4 row)
  let track-o (item 5 row)

  if t != 0 [
    set angular-speed (track-o - orientation) / (clock - t)
  ]

  set t clock
  set x-coordinate track-x
  set y-coordinate track-y
  set orientation track-o
  set linear-speed track-v

  setxy (to-world-x track-x) (to-world-y track-y)
  set heading to-heading track-o
end

;; Update the state of the simulated wheelchair convoy.
to update-convoy [convoy-guide]
  ;; If there is no convoy associated to this guide, create it.
  if not any? wheelchairs with [target = convoy-guide] [
    create-convoy convoy-guide
  ]

  ;; If the convoy is clear, update wheelchair poses.
  if not collision? convoy-guide [
    (run update-convoy-method convoy-guide)
    update-collision-count convoy-guide
    stop
  ]

  ;; If a collision was detected at creation time, delete the convoy.
  ;; This prevents spamming the collision count when wheelchairs get stuck.
  ;; The convoy will be re-created after the guide moves to its next pose.
  let wheelchair-target convoy-guide
  repeat wheelchair-count [
    ask wheelchairs with [target = wheelchair-target] [
      set wheelchair-target self
      die
    ]
  ]
end

;; Create a convoy of wheelchairs behind a guide.
to create-convoy [convoy-guide]
  let wheelchair-target convoy-guide

  create-wheelchairs wheelchair-count [
    set target wheelchair-target
    set color green
    set size 10

    ;; Set wheelchair position and heading to sensible start values.
    ;; This will be overwritten by the update function after all wheelchairs are created.
    set heading ([heading] of target)
    set xcor ([xcor] of target) - (1.0 / map-resolution) * (sin heading)
    set ycor ([ycor] of target) - (1.0 / map-resolution) * (cos heading)

    ;; Set this wheelchair as the target for the next one.
    set wheelchair-target self
  ]
end

;; Report if any of the convoy wheelchairs have collided with an obstacle or pedestrian.
to-report collision? [convoy-guide]
  let wheelchair-target convoy-guide

  loop [
    ;; This assumes that any guide / wheelchair has at most one follower.
    let follower (one-of wheelchairs with [target = wheelchair-target])
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
      set wheelchair-target self
    ]

    if collision-found? [
      report true
    ]
  ]
end

;; Update collision counts for the given guide's convoy.
to update-collision-count [convoy-guide]
  let wheelchair-target convoy-guide
  loop [
    ;; This assumes that any guide / wheelchair has at most one follower.
    let follower (one-of wheelchairs with [target = wheelchair-target])
    if follower = nobody [
      stop
    ]

    ask follower [
      if any? patches in-radius (collision-threshold / map-resolution) with [pcolor = 0] [
        set obstacle-collisions (obstacle-collisions + 1)
        ask convoy-guide [
          set collisions (collisions + 1)
        ]
      ]

      if any? other turtles in-radius (collision-threshold / map-resolution) [
        set pedestrian-collisions (pedestrian-collisions + 1)
        ask convoy-guide [
          set collisions (collisions + 1)
        ]
      ]

      ;; Set this wheelchair as the search key for the next one.
      set wheelchair-target self
    ]
  ]
end

;; Update the position of the wheelchairs using a naive algorithm.
;; The wheelchairs simply follow the guide without avoiding anything in their way.
to update-convoy-naive [convoy-guide]
  let wheelchair-target convoy-guide
  loop [
    ;; This assumes that any guide / wheelchair has at most one follower.
    let follower (one-of wheelchairs with [target = wheelchair-target])
    if follower = nobody [
      stop
    ]

    ask follower [
      let target-xcor ([xcor] of target)
      let target-ycor ([ycor] of target)

      let xcor-offset target-xcor - xcor
      let ycor-offset target-ycor - ycor

      set heading atan xcor-offset ycor-offset

      ;; Revert sin / cos since heading's origin is Up instead of Right.
      set xcor target-xcor - (1.0 / map-resolution) * (sin heading)
      set ycor target-ycor - (1.0 / map-resolution) * (cos heading)

      ;; Set this wheelchair as the search key for the next one.
      set wheelchair-target self
    ]
  ]
end

;; Update the position of the wheelchairs using Artificial Potential Fields (APF) to avoid obstacles.
to update-convoy-apf [convoy-guide]
  let wheelchair-target convoy-guide
  loop [
    ;; This assumes that any guide / wheelchair has at most one follower.
    let follower (one-of wheelchairs with [target = wheelchair-target])
    if follower = nobody [
      stop
    ]

    ask follower [
      let target-xcor ([xcor] of target)
      let target-ycor ([ycor] of target)
      let vector (attraction-vector target-xcor target-ycor)

      let wheelchair-x ([xcor] of self)
      let wheelchair-y ([ycor] of self)

      let nearby (other turtles in-radius (collision-far / map-resolution))
      ask nearby [
        set vector (map + vector (repulsion-vector wheelchair-x wheelchair-y xcor ycor))
      ]

      let neighborhood (patches in-radius (collision-far / map-resolution) with [pcolor = 0])
      ask neighborhood [
        set vector (map + vector (repulsion-vector wheelchair-x wheelchair-y pxcor pycor))
      ]

      let scale (collision-far / map-resolution) / (1 + (count neighborhood) + (count nearby))

      let xcor-offset (item 0 vector) * scale
      let ycor-offset (item 1 vector) * scale

      set heading atan xcor-offset ycor-offset

      ;; Revert sin / cos since heading's origin is Up instead of Right.
      carefully [
        setxy (xcor + xcor-offset) (ycor + ycor-offset)
      ] [
        ;; Nothing to do.
      ]

      ;; Set this wheelchair as the search key for the next one.
      set wheelchair-target self
    ]
  ]
end

;; Compute the attraction vector leading a wheelchair to its target.
to-report attraction-vector [target-xcor target-ycor]
  ;; Normalize vector by sensor range.
  let x (target-xcor - xcor) * (map-resolution / collision-far)
  let y (target-ycor - ycor) * (map-resolution / collision-far)
  let d (sqrt (x ^ 2 + y ^ 2))

  let f attraction-gain * d

  report (list (x * f) (y * f))
end

;; Compute the repulsion vector leading a wheelchair away from an obstacle.
to-report repulsion-vector [wheelchair-x wheelchair-y obstacle-x obstacle-y]
  ;; Normalize vector by sensor range.
  let x (wheelchair-x - obstacle-x) * (map-resolution / collision-far)
  let y (wheelchair-y - obstacle-y) * (map-resolution / collision-far)
  let d (sqrt (x ^ 2 + y ^ 2))

  let f collision-gain * ((1.0 / d) - 1.0) / (d ^ 3)

  report (list (x * f) (y * f))
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
    setxy (to-world-x track-x) (to-world-y track-y)
    set heading to-heading track-o
  ]
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
1
1
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
Pedestrians
Time (h)
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
320
195
353
collision-threshold
collision-threshold
0.1
1.0
0.3
0.1
1
NIL
HORIZONTAL

PLOT
906
156
1232
322
Collisions
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
360
195
393
collision-far
collision-far
0.1
2.0
1.0
0.1
1
NIL
HORIZONTAL

SLIDER
10
400
195
433
collision-gain
collision-gain
0.1
10
0.2
0.1
1
NIL
HORIZONTAL

SLIDER
10
440
195
473
attraction-gain
attraction-gain
0.01
10.0
0.1
0.01
1
NIL
HORIZONTAL

CHOOSER
10
70
195
115
method
method
"naive" "APF"
0

SWITCH
10
30
137
63
all-tracks?
all-tracks?
1
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

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
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
