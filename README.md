# Formation Control Testbed

[AirChair](https://elderlab-york-university.github.io/airchair/) is a semi-autonomous human transportation system composed of multiple wheelchairs operating as a convoy. The first wheelchair follows an on-foot human guide, the second wheelchair follows the first, and so on. Each wheelchair independently tracks its target and performs motion planning to follow along while steering clear of obstacles.

The AirChair motion planner, dubbed Mobile Robot Following Controller (MRFC), computes linear and angular speeds based on:

* The wheelchair leader's distance and linear speed;
* The presence and location of stationary and moving obstacles.

This is an instance of formation control, a popular subject in mobile robotics. Therefore the question arises as to how MRFC compares to other solutions found in the academic literature. Ideally this would be evaluated quantitatively, through a simulation that would abstract away other components involved in the actual process of following a leader.

The Formation Control Testbed (FCT) is a [NetLogo](https://ccl.northwestern.edu/netlogo/) model for testing and quantitative comparison of formation control algorithms. The model features the following characteristics:

* A flat spatial environment, divided in cells of either free or occupied space;
* A number of simulated pedestrians moving across this environment, one of which is the convoy's guide;
* One or more autonomous wheelchairs, each following its leader.

The identity and location of each wheelchair's leader, the occupancy status of environment cells, and the locations of other pedestrians are all given by the model, which is also responsible for executing the speed commands received from motion planner instances.

Pedestrian motion is simulated by replaying person tracks generated from the [ATC pedestrian tracking dataset](https://dil.atr.jp/crest2010_HRI/ATC_dataset/). Each track is a sequence of timestamped and ID'd position records representing the motion of a single person through the ATC shopping center. Specific tracks are selected to play the role of guide, and stored in CSV files alongside all other person tracks that span the same time period.

At present the FCT model implements two formation control algorithms:

* A naive leader-following algorithm with no obstacle avoidance capability;
* An algorithm based on Artificial Potential Fields (APF).

Additional algorithms, including the MRFC algorithm that initially motivated the creation of the FCT model, will follow in the near future.

## Setup

1. Clone this repository
2. Install and open [NetLogo](https://ccl.northwestern.edu/netlogo/)
3. Open file `Formation Control Testbed.nlogo` in the NetLogo IDE.

## Usage

On the top-left corner of the interface there are widgets for controlling the number of simulated wheelchairs, the leader-following algorithm, and whether to replay all track CSV files in sequence or select a specific one for replaying. In the latter case a file opening dialog will be shown for selecting the CSV file. Some sample files are available inside the folder `data/tracks`.

On the bottom-left corner there are widgets to configure the `APF` leader-following algorithm (these are all ignored when the `naive` algorithm is selected):

* `sensor-range` determines the farthest distance at which pedestrians and occupied cells are still considered for avoidance;
* `collision-threshold` is the distance under which an obstacle is considered to have collided with a wheelchair;
* `attraction-gain` controls how strongly each wheelchair is pulled towards its leader;
* `repulsion-gain` controls how strongly each wheelchair is pushed away from obstacles.

Finally, on the top-right corner there are the `Setup` and `Go` buttons to control the execution of the model, along with some visualization widgets:

* `Guide ID` displays the ID of the current guide — this is useful to know which file to replay in case of a specially tricky tracks that requires its own tuning;
* `Total Collisions (all guides)` shows the total number of collisions across all guide tracks in the current session. This helps give a general overview of the performance of different algorithms;
* `Pedestrians (per guide)` and `Collisions (per guide)` respectively plot the number of pedestrians present and the accumulated number of collisions as a function of time. In contrast to the `Total Collisions` monitor, these reset each time a new guide track is loaded. Collisions are plotted by total number (black) as well as collisions with pedestrians (red) and static obstacles (blue).

## Notes

The leader-following algorithm is selected from the `method` widget when the `Setup` button is pressed. Changing it while the simulation is running has no further effect; for a change in selection to come into effect the `Setup` button has to be pressed again. This is a known interface limitation, due to how the method selection is implemented by instantiating an anonymous function. In practice this shouldn't be much of a problem, since for testing purposes it makes little sense to change algorithms in the middle of a session.
