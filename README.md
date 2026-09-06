# Traffic Simulator

A C++20 prototype for simulating traffic on a directed road network. The
simulator reads a city graph and demand weights from CSV files, precomputes
routes, generates vehicles, models queues, safe longitudinal movement and
traffic-light phases, then writes a frame-by-frame trace for a browser
visualizer.

## What is implemented

- Directed road graph loaded from CSV files.
- Validation of road endpoints, road IDs, self-loops, positive lengths, and
  positive speed limits.
- Shortest-route successor table based on travel time (`length / maxspeed`).
- Random origin and destination sampling from independent discrete demand
  distributions. A vehicle is resampled while origin equals destination.
- External waiting queue at every intersection, with scheduled spawning and a
  minimum entry headway based on `CAR_LENGTH + SAFETY_GAP`.
- One traffic light per intersection. Its phases serve, in turn, the external
  queue and every directed road that enters that intersection. The phase for
  the external queue has the special road ID `-2`.
- Configurable traffic-light controller: static equal-duration phases or an
  adaptive controller, selected with the `isAdaptive` constant.
- Adaptive green-time allocation based on local detector demand, observed turn
  proportions, and the expected availability of downstream roads.
- One `deque` per directed road. The front vehicle is closest to the road exit;
  the back vehicle is closest to the entry.
- Safe car-following movement: a vehicle cannot advance beyond the required
  gap behind the preceding vehicle.
- Transfer to the next road when there is enough entry space; the unused part
  of the time step is preserved as distance on the next road.
- Individual driver behaviour through distinct impatience thresholds and
  recovery rates. Drivers can abandon their planned next road after prolonged
  blockage and choose an available alternative.
- Protection against processing a transferred vehicle twice in the same time
  step (`lastStepProcessed`).
- Vehicle reinitialization after reaching its destination, followed by a new
  origin/destination assignment and placement in an external waiting queue.
  Reinitialization is deferred until the end of the step, so the same vehicle
  cannot be processed again in that step.
- Per-step moving/stationary vehicle statistics, both as whole-vehicle counts
  and as fractions of a time step.
- Completed-trip delay statistics for the external queue, the internal road
  network, and the complete trip.
- A browser visualizer with step controls, a slider, automatic playback,
  directed-road arrows, separate lanes for two-way roads, traffic-light phase
  markers, and a fixed statistics panel.

## Current model and limits

The model has one lane per directed road. A pair of opposite directed roads is
drawn as a two-way road by the visualizer. At an intersection, one incoming
road or the external queue has green in the current phase. It does not yet
model multiple lanes, turning lanes, pedestrians, yellow/all-red intervals,
or simultaneous non-conflicting movements.

Roads are processed in index order. Consequently, a vehicle can be denied
entry to a target road that becomes free later in the same time step. This is
an order-dependent approximation, not a synchronized intersection solver.

The route precomputation currently uses repeated Dijkstra-style searches with
linear minimum selection. It is suitable for the small example network but
should be replaced with `priority_queue` Dijkstra before scaling to large
networks.

Traffic-light durations can be static or adaptive. In both modes the phase
order is fixed and exactly one incoming road (or the external queue) is green
at an intersection. Adaptive control changes only phase durations; it does
not change the vehicle-movement rules.

### Individual drivers: patience, rerouting, and learning

Every vehicle has two persistent behavioural traits and one per-trip state:

- `impatienceThreshold`: the accumulated impatience at which it may reroute;
- `currentImpatience`: its current frustration level, reset for each trip;
- `patienceRegen`: the fraction by which its impatience is reduced after a
  successful reroute.

At initialization, thresholds are sampled from a bounded normal distribution
centred on `MEAN_IMPATIENCE_THRESHOLD`, while regeneration factors are sampled
independently around `MEAN_PATIENCE_REGENERATION`. This makes drivers react at
different times instead of treating every vehicle identically.

Impatience grows only when a vehicle is prevented from using a green movement:
when it is close to a green internal intersection but constrained by the
vehicle ahead, when it reaches a green intersection whose chosen outgoing road
has insufficient entry space, or when the head of an external queue has green
but cannot enter its first road. The increase is proportional to the blocked
part of the time step and is amplified by `IMPATIENCE_MULTIPLIER`.

If the threshold is exceeded, the driver may make a non-optimal routing
decision. It evaluates outgoing roads from the current intersection, rejects
roads without sufficient entry space, the originally planned next road, and
choices that immediately lead back toward the intersection just left. Among
the remaining candidates it chooses the one with the smallest free-flow
remaining travel time. Therefore a reroute can be slower than the original
shortest route: it represents a frustrated driver trying to escape a local
blockage, not a globally omniscient route planner.

Rerouting can decongest the blocked approach by removing a vehicle that would
otherwise keep waiting for the same movement, which may also let following
vehicles progress. It is not guaranteed to improve the global result: the
alternative can be longer or can transfer congestion to another road. Its
effect is therefore measured through the completed-trip and movement
statistics rather than assumed by the model.

The vehicle records every road it chose through such a reroute in
`forbiddenRoads`. It will not select the same road again during that trip.
This is the model's limited form of driver learning: a driver does not repeat a
previous rerouting choice after it has already tried it. The rerouting memory,
impatience, and current flag are reset when the vehicle finishes its trip and
is reinitialized; its threshold and regeneration trait remain individual to
that vehicle.

Impatience also recovers. For an internal road movement representing at least
90% of a simulation step, it decreases by `fullStepImpatienceReduction`; a
successful reroute applies the driver's multiplicative `patienceRegen`
reduction. A vehicle row
in the trace has `isImpatient = 1` when its accumulated impatience is above
its threshold, or in the exact step where it reroutes. The visualizer can show
this state with either a red marker or an emoji.

### Adaptive traffic lights

Set the constant at the top of `src/main.cpp` to choose the controller:

```cpp
#define isAdaptive 0  // static baseline: defaultGreenSteps for every phase
#define isAdaptive 1  // connected adaptive controller
```

The adaptive controller preserves each signal's total cycle length:

```text
number of phases × defaultGreenSteps
```

Each phase first receives `minGreenSteps`. The remaining steps are distributed
proportionally to its effective score:

```text
effectiveScore = detectorScore × expectedDownstreamAvailability
```

`detectorScore` is a distance-weighted count of vehicles within the detection
radius of the intersection. Snapshots are collected globally every
`defaultGreenSteps`, immediately before any cycle boundary that can use them.

For each incoming phase, `roadChangeMatrix` records the observed outcomes:
the virtual external exit (column `0`) or each outgoing road. These counts
estimate turn probabilities. The expected downstream availability is their
weighted average: an exiting vehicle contributes availability `1`, while a
vehicle continuing on an outgoing road is discounted when that road's
downstream detector is congested. Thus a phase with high local demand can
still receive less green if it would probably feed a blocked road.

For temporary inspection, `output/trafficlights.out` records, at each adaptive
cycle update, the step, phase road ID, detector score, and expected downstream
availability. It is a debug trace; `output/output.out` remains the trace used
by the visualizer.

## Repository layout

```text
.
|-- CMakeLists.txt
|-- README.md
|-- start_visualizer.bat        # Starts a local server and opens the viewer
|-- visualizer.html             # Browser-based trace viewer
|-- data/
|   |-- intersections.csv       # Nodes and external-entry speeds
|   |-- roads.csv               # Directed road segments
|   `-- demand.csv              # Origin and destination weights
|-- output/
|   |-- output.out              # Generated simulation trace
|   `-- trafficlights.out       # Adaptive-controller debug trace
`-- src/
    `-- main.cpp
```

## Requirements

### Simulator

- CMake 3.20 or newer
- A C++20 compiler

### Visualizer

- A modern browser
- Python 3 on Windows (only to serve the local files)

The visualizer has no package manager dependencies and does not require an
internet connection.

## Build and run the simulator

The commands below are run from the repository root.

```powershell
cmake -S . -B build
cmake --build build --config Debug
.\build\Debug\simulator.exe
```

For a single-config generator such as Ninja, the executable is normally:

```powershell
.\build\simulator.exe
```

The program prints basic progress to the terminal and overwrites
`output/output.out` with the new trace. The `output/` directory must be present
in the repository; Git cannot retain an empty directory.

In Visual Studio, opening the project folder lets CMake configure the project.
Build and run the `simulator` target from the CMake Targets view. Do not rely
on a stale `src/main.exe`; CMake builds the current executable under its build
directory.

## Run the visualizer

1. Run the simulator to generate `output/output.out`.
2. Double-click `start_visualizer.bat`.
3. The script starts a Python server at `http://localhost:8000` and opens
   `http://localhost:8000/visualizer.html`.
4. Use **Step back**, **Step ahead**, the slider, **Run**, and **Pause** to
   inspect the recorded frames.

The server is bound to `127.0.0.1`, so it accepts connections only from the
same computer. If the port is already occupied by an existing viewer server,
the script reuses it instead of opening another one.

After generating a new trace, refresh the browser page to load the new
`output/output.out` file. The road network CSV files are checked periodically
and the displayed graph is rebuilt when they change.

Opening `visualizer.html` directly from the file system does not work because
browsers block JavaScript from reading nearby local files automatically. The
local server is required for the viewer to load the CSV files and trace.

## Input files

### `data/intersections.csv`

```csv
x,y,external_speed,id
0,0,10,0
```

| Field | Meaning |
| --- | --- |
| `x`, `y` | Coordinates used by the visualizer. |
| `external_speed` | Entry speed used to schedule vehicles waiting outside this intersection. |
| `id` | Intersection identifier. IDs must be contiguous from `0` to `N - 1`. |

### `data/roads.csv`

```csv
from,to,lg,maxspeed,id
0,1,30,10,0
```

Each row is one directed road.

| Field | Meaning |
| --- | --- |
| `from`, `to` | Source and target intersection IDs. |
| `lg` | Positive road length. |
| `maxspeed` | Positive speed limit. |
| `id` | Unique road identifier. IDs must match their row index in the current implementation. |

For a two-way street, provide two rows: one for `A -> B` and one for `B -> A`.

### `data/demand.csv`

```csv
index,interest_weight,departure_weight
0,10,10
```

| Field | Meaning |
| --- | --- |
| `index` | Intersection ID; rows must be ordered by this ID. |
| `interest_weight` | Relative probability of choosing the intersection as a destination. |
| `departure_weight` | Relative probability of choosing the intersection as an origin. |

Weights may be zero, but each distribution needs at least one positive weight.

## Simulation defaults

These constants are currently defined at the top of `src/main.cpp`:

| Setting | Value |
| --- | ---: |
| Number of simulated vehicles | `400` |
| Number of time steps | `4000` |
| Time-step duration | `0.1` |
| Vehicle length | `1` |
| Safety gap | `1.5` |
| Default green duration per phase | `20` steps |
| Minimum adaptive green duration | `10` steps |
| Adaptive controller enabled | `isAdaptive = 1` |
| Random seed | `100` |

The fixed seed makes runs deterministic while the input data and constants are
unchanged.

## Statistics

### Per-step movement

Each frame contains four movement values:

| Field | Meaning |
| --- | --- |
| `MovingVehiclesNo` | Number of vehicles that moved by a meaningful positive amount during this step. A vehicle that reaches its destination is also counted as moving. |
| `StationaryVehiclesNo` | Number of vehicles that did not move during this step. |
| `fMovingNo` | Sum of the fractions of the time step during which vehicles moved. For example, a vehicle moving for half of the step contributes `0.5`. |
| `fStationaryNo` | Sum of the complementary stationary fractions of the time step. |

The integer fields answer “how many vehicles moved at all?”, while the
fractional fields preserve partial movement within a step. When a vehicle
finishes a trip before the end of the step, its remaining time is not counted
as motion for that completed trip; therefore the fractional totals are not
required to equal the active-vehicle count exactly.

### Completed-trip delay

Statistics are updated when a vehicle reaches its destination. For a trip:

| Segment | Expected time | Actual time |
| --- | --- | --- |
| External | `expectedExternalTime` | `actualSpawnTime - initTime` |
| Internal | shortest-path time from `timeMatrix` | `endTime - actualSpawnTime` |
| Total | external expected + internal expected | `endTime - initTime` |

`expectedExternalTime` is a free-flow queue baseline: the number of vehicles
already ahead of the vehicle, multiplied by the entry headway. The internal
expected time is the shortest free-flow travel time, using `length / maxspeed`
for each road. Thus, a ratio of `1.25` means the actual time was 25% above its
expected baseline.

For each of the external, internal, and total segments, the trace stores two
different aggregates:

| Aggregate | Formula | Interpretation |
| --- | --- | --- |
| Average vehicle | `sum(actual_i / expected_i) / N` | Typical completed vehicle: every trip has equal weight. |
| Time-weighted | `sum(actual_i) / sum(expected_i)` | Global delay: trips with larger expected durations have proportionally more weight. |

Both are useful and neither replaces the other. The average-vehicle value is
better for describing what a typical driver experiences; the time-weighted
value is better for the overall travel-time burden of the system. Trip metrics
are emitted only after completed trips exist. Initial vehicles with an initial
`spawnTime` of zero are excluded by the current implementation, because their
external expected time can be zero.

The visualizer displays these six ratios as percentages. Up to 150% is green,
150–250% is yellow, and over 250% is red. These colours are display thresholds,
not additional simulation rules.

## Trace format

`output/output.out` is a whitespace-delimited trace, despite the `.out`
extension. Its first line is the number of frames. Each frame contains, in
order:

1. Completed-trip count and six trip ratios, or `completed_trip_count 0` when
   no trip data exists yet.
2. The four per-step movement values.
3. One traffic-light phase ID per intersection. `-2` means that intersection's
   external queue has green; any non-negative ID is the incoming road with
   green.
4. The active-vehicle count.
5. One record per active vehicle.

For example, the initial frame may look like this:

```text
2000
0 0
0 0 0 0
-2 -2 -2 -2
5
0 2 1 5 0.03 0
1 4 5 14 0.05 0
...
```

When trip data exists, its first frame line has seven fields:

```text
completed_trips  avg_external  avg_internal  avg_total  weighted_external  weighted_internal  weighted_total
```

Each vehicle row has:

```text
vehicle_id  from_intersection  to_intersection  road_id  normalized_position  is_impatient
```

`normalized_position` is in `[0, 1]`: `0` is the entry of the directed road
and `1` is its exit. The visualizer uses `road_id` and this position to place
the vehicle on the correct directional lane. The `from` and `to` fields are
also useful for checking trace consistency. `is_impatient` is `1` when the
driver is currently above its rerouting threshold or rerouted during that
frame; otherwise it is `0`.

## Next development steps

- Replace the all-pairs route precomputation with priority-queue Dijkstra and make code optimisations (time and memory)
- Use a synchronized transfer/reservation phase to remove road-order effects.
- Add automated tests for route validity, queue spacing, and trace format.
- Compare static, simple demand-adaptive, connected adaptive, and
  reinforcement-learning traffic-light controllers using the recorded metrics.
- Implement complex intersetions and multiple traffic lanes per road
