#include "algorithms.h"
#include "pq.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *algo_name(Algorithm a)
{
    static const char *names[] = {"Dijkstra (heap)", "A* search", "BFS (min hops)",
                                  "Bellman-Ford", "Dijkstra (array)"};
    return names[a];
}

const char *algo_key(Algorithm a)
{
    static const char *keys[] = {"dijkstra", "astar", "bfs", "bellman", "dijkstra_array"};
    return keys[a];
}

/* ------------------------------------------------------------ helpers */

typedef struct {
    double *dist;
    int    *prev;      /* predecessor node on the best path */
    char   *done;
} Search;

static Search search_init(int n)
{
    Search s = {xmalloc(sizeof(double) * n), xmalloc(sizeof(int) * n), xcalloc(n, 1)};
    for (int i = 0; i < n; i++) {
        s.dist[i] = INF;
        s.prev[i] = -1;
    }
    return s;
}

static void search_free(Search *s)
{
    free(s->dist);
    free(s->prev);
    free(s->done);
}

static void record(PathResult *r, int v)
{
    if (r->visit_order) r->visit_order[r->visit_count++] = v;
    r->visited++;
}

/* Walk prev[] back from dst and reverse it into r->path. */
static void build_path(const Graph *g, PathResult *r, const Search *s, int src, int dst)
{
    if (src != dst && s->prev[dst] < 0) return;
    int len = 0;
    for (int v = dst; v != -1; v = s->prev[v]) {
        r->path[len++] = v;
        if (v == src) break;
    }
    for (int i = 0; i < len / 2; i++) {
        int t                 = r->path[i];
        r->path[i]            = r->path[len - 1 - i];
        r->path[len - 1 - i]  = t;
    }
    r->path_len   = len;
    r->found      = 1;
    r->distance_m = path_distance(g, r->path, len);
    r->time_s     = path_time(g, r->path, len);
}

/* Admissible heuristic: straight-line distance never overestimates road
 * distance, and straight-line / top speed never overestimates time. */
static double heuristic(const Graph *g, int v, int dst, WeightMode mode)
{
    double d = euclid(g->nodes[v].x, g->nodes[v].y, g->nodes[dst].x, g->nodes[dst].y) * METERS_PER_UNIT;
    return mode == WEIGHT_DISTANCE ? d : d / g->max_speed;
}

/* ------------------------------------------------------------ algorithms */

/* Dijkstra and A* share one loop: A* only adds h(v) to the heap key. */
static void run_best_first(const Graph *g, int src, int dst, WeightMode mode, int use_heuristic, PathResult *r)
{
    Search   s  = search_init(g->n_nodes);
    MinHeap *pq = pq_create(g->n_nodes);

    s.dist[src] = 0;
    pq_push(pq, src, use_heuristic ? heuristic(g, src, dst, mode) : 0);

    while (!pq_empty(pq)) {
        int u     = pq_pop(pq);
        s.done[u] = 1;
        record(r, u);
        if (u == dst) break;

        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            int    v = g->edges[a->edge].to;
            double w = edge_weight(g, a->edge, mode);
            r->relaxations++;
            if (s.done[v] || w >= INF) continue;
            double nd = s.dist[u] + w;
            if (nd < s.dist[v]) {
                s.dist[v] = nd;
                s.prev[v] = u;
                pq_push_or_decrease(pq, v, nd + (use_heuristic ? heuristic(g, v, dst, mode) : 0));
            }
        }
    }
    r->pq_ops = pq->ops;
    if (s.dist[dst] < INF) {
        r->cost = s.dist[dst];
        build_path(g, r, &s, src, dst);
    }
    pq_free(pq);
    search_free(&s);
}

/* O(V^2): no priority queue, scan every node to find the next minimum. */
static void run_dijkstra_array(const Graph *g, int src, int dst, WeightMode mode, PathResult *r)
{
    Search s    = search_init(g->n_nodes);
    s.dist[src] = 0;

    for (int iter = 0; iter < g->n_nodes; iter++) {
        int u = -1;
        for (int v = 0; v < g->n_nodes; v++)
            if (!s.done[v] && s.dist[v] < INF && (u < 0 || s.dist[v] < s.dist[u])) u = v;
        if (u < 0) break;
        s.done[u] = 1;
        record(r, u);
        if (u == dst) break;
        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            int    v = g->edges[a->edge].to;
            double w = edge_weight(g, a->edge, mode);
            r->relaxations++;
            if (!s.done[v] && w < INF && s.dist[u] + w < s.dist[v]) {
                s.dist[v] = s.dist[u] + w;
                s.prev[v] = u;
            }
        }
    }
    if (s.dist[dst] < INF) {
        r->cost = s.dist[dst];
        build_path(g, r, &s, src, dst);
    }
    search_free(&s);
}

/* Breadth-first search: shortest path in number of intersections. */
static void run_bfs(const Graph *g, int src, int dst, WeightMode mode, PathResult *r)
{
    Search s = search_init(g->n_nodes);
    Queue *q = queue_create(g->n_nodes);

    s.done[src] = 1;
    s.dist[src] = 0;
    queue_push(q, src);
    while (q->size) {
        int u = queue_pop(q);
        record(r, u);
        if (u == dst) break;
        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            int v = g->edges[a->edge].to;
            r->relaxations++;
            if (s.done[v] || g->edges[a->edge].closed) continue;
            s.done[v] = 1;
            s.dist[v] = s.dist[u] + 1;
            s.prev[v] = u;
            queue_push(q, v);
        }
    }
    if (s.done[dst]) {
        build_path(g, r, &s, src, dst);
        r->cost = mode == WEIGHT_DISTANCE ? r->distance_m : r->time_s;
    }
    queue_free(q);
    search_free(&s);
}

/* Bellman-Ford: relax every edge up to V-1 times; stop early once a full
 * pass changes nothing. Handles negative weights (roads never have them,
 * but it is the reference algorithm for correctness checks). */
static void run_bellman_ford(const Graph *g, int src, int dst, WeightMode mode, PathResult *r)
{
    Search s    = search_init(g->n_nodes);
    s.dist[src] = 0;
    record(r, src);
    s.done[src] = 1;

    for (int pass = 0; pass < g->n_nodes - 1; pass++) {
        int changed = 0;
        for (int e = 0; e < g->n_edges; e++) {
            int u = g->edges[e].from, v = g->edges[e].to;
            r->relaxations++;
            if (s.dist[u] >= INF) continue;
            double w = edge_weight(g, e, mode);
            if (w >= INF) continue;
            if (s.dist[u] + w < s.dist[v]) {
                s.dist[v] = s.dist[u] + w;
                s.prev[v] = u;
                changed   = 1;
                if (!s.done[v]) { /* first time this node is reached */
                    s.done[v] = 1;
                    record(r, v);
                }
            }
        }
        if (!changed) break;
    }
    if (s.dist[dst] < INF) {
        r->cost = s.dist[dst];
        build_path(g, r, &s, src, dst);
    }
    search_free(&s);
}

PathResult find_path(const Graph *g, int src, int dst, WeightMode mode, Algorithm algo, int record_visits)
{
    PathResult r;
    memset(&r, 0, sizeof r);
    r.algo        = algo;
    r.cost        = INF;
    r.path        = xmalloc(sizeof(int) * g->n_nodes);
    r.visit_order = record_visits ? xmalloc(sizeof(int) * g->n_nodes) : NULL;

    double t0 = now_us();
    switch (algo) {
    case ALGO_DIJKSTRA:       run_best_first(g, src, dst, mode, 0, &r); break;
    case ALGO_ASTAR:          run_best_first(g, src, dst, mode, 1, &r); break;
    case ALGO_BFS:            run_bfs(g, src, dst, mode, &r); break;
    case ALGO_BELLMAN_FORD:   run_bellman_ford(g, src, dst, mode, &r); break;
    case ALGO_DIJKSTRA_ARRAY: run_dijkstra_array(g, src, dst, mode, &r); break;
    default: break;
    }
    r.runtime_us = now_us() - t0;
    return r;
}

void path_result_free(PathResult *r)
{
    free(r->path);
    free(r->visit_order);
    r->path        = NULL;
    r->visit_order = NULL;
}

double path_distance(const Graph *g, const int *path, int len)
{
    double d = 0;
    for (int i = 0; i + 1 < len; i++) d += g->edges[graph_find_edge(g, path[i], path[i + 1])].length;
    return d;
}

double path_time(const Graph *g, const int *path, int len)
{
    double t = 0;
    for (int i = 0; i + 1 < len; i++) t += edge_travel_time(g, graph_find_edge(g, path[i], path[i + 1]));
    return t;
}

void path_print(const Graph *g, const PathResult *r)
{
    if (!r->found) {
        printf("  %s: no route found\n", algo_name(r->algo));
        return;
    }
    printf("  Route: ");
    for (int i = 0; i < r->path_len; i++)
        printf("%s%s", g->nodes[r->path[i]].name, i + 1 < r->path_len ? " -> " : "\n");
    printf("  Distance %.2f km | ETA %.1f min | %d intersections | %d nodes explored | %.1f us\n",
           r->distance_m / 1000, r->time_s / 60, r->path_len, r->visited, r->runtime_us);
}
