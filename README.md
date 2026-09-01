# Traffic Simulator

A C++20 prototype for simulating traffic on a directed road network. The
simulator reads a city graph and demand weights from CSV files, precomputes
routes, generates vehicles, models queues and safe longitudinal movement, and
writes a frame-by-frame trace that can be explored in a lightweight browser
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
- One `deque` per directed road. The front vehicle is closest to the road exit;
  the back vehicle is closest to the entry.
- Safe car-following movement: a vehicle cannot advance beyond the required
  gap behind the preceding vehicle.
- Transfer to the next road when there is enough entry space; the unused part
  of the time step is preserved as distance on the next road.
- Protection against processing a transferred vehicle twice in the same time
  step (`lastStepProcessed`).
- Vehicle reinitialization after reaching its destination, followed by a new
  origin/destination assignment and placement in an external waiting queue.
- A browser visualizer with step controls, a slider, automatic playback,
  directed-road arrows, and separate lanes for two-way roads.

## Current model and limits

The model has one lane per directed road. A pair of opposite directed roads is
drawn as a two-way road by the visualizer, but lane selection, turning lanes,
traffic lights, intersection occupancy, and conflict resolution between
movements are not modeled yet.

Roads are processed in index order. Consequently, a vehicle can be denied
entry to a target road that becomes free later in the same time step. This is
an order-dependent approximation, not a synchronized intersection solver.

The route precomputation currently uses repeated Dijkstra-style searches with
linear minimum selection. It is suitable for the small example network but
should be replaced with `priority_queue` Dijkstra before scaling to large
networks.

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
|   `-- output.out              # Generated simulation trace
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
`output/output.out` with the new data. The `output/` directory is present in 
the reposirory, along with a sample output.out file.

In Visual Studio, opening the project folder lets CMake configure the project.
Build and run the `simulator` target from the CMake Targets view.  CMake builds 
the current executable under its build directory.

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
| Number of time steps | `1000` |
| Time-step duration | `0.1` |
| Vehicle length | `1` |
| Safety gap | `1.5` |
| Random seed | `100` |

The fixed seed makes runs deterministic while the input data and constants are
unchanged. Changing the constants generates new traffic patterns.

## Trace format

`output/output.out` is a whitespace-delimited trace. Its first line is the number of frames. Each frame then contains the
number of active vehicles followed by one row per active vehicle:

```text
1000
5
0 2 1 5 0.03
1 4 5 14 0.05
...
```

Each vehicle row has:

```text
vehicle_id  from_intersection  to_intersection  road_id  normalized_position
```

`normalized_position` is in `[0, 1]`: `0` is the entry of the directed road
and `1` is its exit. The visualizer uses `road_id` and this position to place
the vehicle on the correct directional lane. The `from` and `to` fields are
also useful for checking trace consistency.

## Next development steps

- Replace the all-pairs route precomputation with priority-queue Dijkstra.
- Add traffic-light phases and intersection movement conflicts.
- Model multiple lanes and turning movements.
- Use a synchronized transfer/reservation phase to remove road-order effects.
- Add automated tests for route validity, queue spacing, and trace format.
