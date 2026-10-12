# Smart City Traffic Intelligence System

DSA project in **pure C**: a city road network as a graph, shortest-path routing with
a priority queue, a traffic-signal simulation, and an ambulance "green corridor" — plus
an optional browser dashboard that visualises everything the C program computes.

```
Layer 1  Core engine (C)        graph · min-heap PQ · Dijkstra / A* / BFS / Bellman-Ford · simulation
Layer 2  Visualisation (web/)   roads, vehicles, traffic lights, ambulance movement, route search
Layer 3  Analytics (C + web/)   distance, nodes visited, congestion, algorithm comparison, scaling
```

The web layer contains **no algorithms** — it only draws `web/data/sim_data.js`, which the
C program writes. If your college wants a C-only submission, `src/` + `include/` + `data/`
is the complete project; the dashboard is optional.

## Run it

```bash
cd smart-city-traffic
make            # builds ./traffic  (gcc, C99, only needs libm)
./traffic       # interactive menu
make export     # runs everything and writes web/data/sim_data.js
```

Then open `web/index.html` in any browser (double-click works; no server needed).

### Menu

```
1. Show city map (adjacency list)
2. Find a route (pick algorithm)          -> path, distance, ETA, exploration order
3. Compare all algorithms on one route
4. Close / reopen a road                   -> accident / roadwork, routes avoid it
5. Run traffic + ambulance simulation      -> with vs without green corridor
6. Algorithm analytics over all routes     -> all 552 src/dst pairs
7. Scaling benchmark (100 -> 10,000 nodes)
8. Export everything for the web dashboard
```

## Project layout

```
include/            headers (one per module)
src/
  graph.c           adjacency-list graph, map loader, grid generator, BPR congestion model
  pq.c              indexed binary min-heap (decrease-key) + circular FIFO queue
  algorithms.c      Dijkstra (heap), Dijkstra (O(V^2) array), A*, BFS, Bellman-Ford
  traffic.c         signals, vehicles, stop-line queues, ambulance pre-emption
  analytics.c       all-pairs comparison, scaling benchmark
  export.c          writes web/data/sim_data.js
  main.c            CLI menu
data/city_map.txt   24 intersections, 41 two-way roads (editable)
web/                index.html + style.css + app.js (dashboard)
```

## Data structures and algorithms used

| Concept | Where | Why |
|---|---|---|
| **Graph — adjacency list** (array of linked lists) | `graph.c` | Sparse road network, O(V+E) memory; each road stores length, lanes, speed, capacity, live load |
| **Indexed min-heap** with `pos[]` map | `pq.c` | O(log V) push / pop / **decrease-key** for Dijkstra and A* |
| **Circular queue** | `pq.c` | BFS frontier, and every traffic light's stop line (FIFO discharge) |
| **Dijkstra (heap)** | `algorithms.c` | O((V+E) log V) optimal routes |
| **Dijkstra (array)** | `algorithms.c` | O(V²) textbook version — shows why the PQ matters |
| **A\*** | `algorithms.c` | Dijkstra + straight-line heuristic (admissible ⇒ still optimal, explores fewer nodes) |
| **BFS** | `algorithms.c` | Fewest intersections; ignores weights, so not always shortest |
| **Bellman-Ford** | `algorithms.c` | O(V·E) relaxation with early exit; correctness reference |
| **BPR function** `t = t0·(1 + 0.15·(v/c)^4)` | `graph.c` | Standard traffic-engineering congestion → travel time |

### Simulation model (`traffic.c`)

- Every intersection with ≥3 approaches gets a **2-phase signal** (E-W / N-S), 18 s green + 3 s yellow, staggered offsets.
- Vehicles pick routes with **Dijkstra on live travel time**; speed on a road follows the BPR curve.
- At a red light vehicles join the road's **FIFO stop-line queue** and leave at saturation flow (0.5 veh/s/lane).
- Half of all trips head to landmarks (station, mall, …) → rush-hour hot spots in the centre.
- The **ambulance** plans with **A\*** on travel time. With the *green corridor* on, each signal ahead turns green
  for it when it is within 300 m and other traffic pulls over.
- Both runs use the same seed, so every other car is identical — the comparison is fair.

## Sample results (from `make export`)

```
Ambulance City Hospital -> Stadium
  without corridor: 318 s (24 s stopped at reds)
  with corridor:    250 s (0 s stopped, 4 signals pre-empted)   -> 21% faster

All 552 routes, distance weights   Visited  Relaxed  Optimal
  Dijkstra (heap)                     13.0     43.7     100%
  A* search                            4.9     15.4     100%
  BFS (min hops)                      13.0     43.7      70%  (+1.28% longer on average)
  Bellman-Ford                        24.0    382.7     100%
```

Timings in the scaling benchmark depend on your machine; the trend (array Dijkstra growing
as V² while the heap version stays near-linear) is what to look at.

## Editing the map

`data/city_map.txt`:

```
NODE <id> <x> <y> <junction|hospital|landmark> <Name_with_underscores>
ROAD <u> <v> <lanes> <speed_kmh>        # two-way, length = straight line × 5 m
```

Ambulance start/end and simulation parameters are in `sim_default_config()` in `src/traffic.c`.
