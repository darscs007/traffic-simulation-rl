# Traffic Simulator

Prototype C++ traffic simulator for a directed road network. The program loads a city graph and traffic-demand weights from CSV files, computes shortest routes, then simulates vehicle movement along those routes.

## Current functionality

- Loads intersections and directed roads from CSV files.
- Validates road input, including missing intersections, duplicate road IDs, non-positive lengths/speeds, and self-loops.
- Computes all-pairs route successors using repeated Dijkstra-style shortest-path searches. Edge cost is travel time: `length / maxspeed`.
- Samples each vehicle's origin and destination using weighted demand distributions.
- Simulates 100 vehicles for 1,000 time steps of 0.1 seconds, using deterministic random seed `42`.
- Prints each active vehicle's interpolated `(x, y)` position and current road ID to standard output.

## Project layout

```text
.
├── CMakeLists.txt
├── data/
│   ├── demand.csv          # Origin/destination sampling weights
│   ├── intersections.csv   # Intersection coordinates and IDs
│   └── roads.csv           # Directed road segments
└── src/
    └── main.cpp
```

## Requirements

- CMake 3.20 or newer
- A C++20 compiler (for example MSYS2 UCRT64 `g++`)

No third-party libraries are required.

## Build and run

From the repository root:

```bash
cmake -S . -B build
cmake --build build
```

Run the produced executable. With a single-config generator such as Ninja:

```bash
./build/simulator
```

With a Visual Studio multi-config generator, the executable is normally under `build/Debug/simulator.exe` after building the `Debug` configuration.

## Input data

### `data/intersections.csv`

```csv
x,y,id
0,0,0
```

- `x`, `y`: intersection coordinates
- `id`: unique non-negative intersection identifier

### `data/roads.csv`

```csv
from,to,lg,maxspeed,id
0,1,30,10,0
```

- `from`, `to`: source and destination intersection IDs
- `lg`: road length; must be positive
- `maxspeed`: speed limit; must be positive
- `id`: unique road identifier

Each row represents one directed road. Add both `A,B,...` and `B,A,...` rows for a two-way road.

### `data/demand.csv`

```csv
index,interest_weight,departure_weight
0,10,10
```

- `interest_weight`: relative destination-selection weight
- `departure_weight`: relative origin-selection weight

At present, the data rows must be ordered by intersection ID and cover IDs from `0` through `N - 1`, because weights are stored by row position.

## Current limitations

- The simulator models free-flow movement only. Road capacity, queues, collision avoidance, traffic lights, and congestion effects are not implemented yet.
- Road queues and several vehicle fields exist in the code but are not yet used by the simulation logic.
- Unreachable origin-destination pairs are not handled explicitly.
- Output is textual; there is no visualization or metrics summary yet.
- Shortest paths are precomputed with a straightforward implementation and will need optimization for large networks.

## Suggested next steps

1. Add handling and reporting for disconnected pairs of intersections.
2. Model road occupancy, capacity, and intersection queues.
3. Export simulation data and add summary metrics such as travel time and waiting time.
4. Add tests for CSV validation and routing correctness.
