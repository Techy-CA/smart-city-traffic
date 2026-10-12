#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"

typedef enum {
    ALGO_DIJKSTRA = 0,    /* binary-heap priority queue, O((V+E) log V) */
    ALGO_ASTAR,           /* Dijkstra + admissible straight-line heuristic */
    ALGO_BFS,             /* fewest intersections, ignores weights, O(V+E) */
    ALGO_BELLMAN_FORD,    /* edge relaxation, O(V*E), early exit        */
    ALGO_DIJKSTRA_ARRAY,  /* textbook O(V^2) Dijkstra, linear min scan  */
    ALGO_COUNT
} Algorithm;

typedef struct {
    Algorithm algo;
    int       found;
    int      *path;          /* node ids src..dst                         */
    int       path_len;
    double    cost;          /* in the requested weight unit              */
    double    distance_m;    /* real length of the returned path          */
    double    time_s;        /* travel time of the path at current load   */
    int       visited;       /* nodes expanded (settled / dequeued)       */
    long      relaxations;   /* edge relaxations attempted                */
    long      pq_ops;        /* heap push + decrease-key                  */
    double    runtime_us;
    int      *visit_order;   /* nodes in the order they were expanded     */
    int       visit_count;
} PathResult;

const char *algo_name(Algorithm a);
const char *algo_key(Algorithm a);   /* short machine-readable id */

/* Runs `algo` from src to dst. `record_visits` stores expansion order (for
 * the visualisation); turn it off for timing benchmarks. */
PathResult find_path(const Graph *g, int src, int dst, WeightMode mode, Algorithm algo, int record_visits);
void       path_result_free(PathResult *r);
void       path_print(const Graph *g, const PathResult *r);

double path_distance(const Graph *g, const int *path, int len);
double path_time(const Graph *g, const int *path, int len);

#endif
