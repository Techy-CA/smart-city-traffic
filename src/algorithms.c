#include "city.h"

#include <stdlib.h>
#include <string.h>

const char *ALGO_NAME[ALGO_COUNT] = {"Dijkstra (heap)", "A* search", "BFS (min hops)", "Bellman-Ford", "Dijkstra (array)"};
const char *ALGO_KEY[ALGO_COUNT] = {"dijkstra", "astar", "bfs", "bellman", "dijkstra_array"};

static double dist[MAX_NODES];
static int prev[MAX_NODES];
static char done[MAX_NODES];

static void visit(PathResult *r, int v)
{
    r->order[r->visited] = v;
    r->visited++;
}

static double heuristic(const Graph *g, int v, int dst, WeightMode mode)
{
    double d = euclid(g->nodes[v].x, g->nodes[v].y, g->nodes[dst].x, g->nodes[dst].y) * METERS_PER_UNIT;
    if (mode == BY_DISTANCE) return d;
    return d / g->max_speed;
}

static void best_first(const Graph *g, int src, int dst, WeightMode mode, int use_heuristic, PathResult *r)
{
    MinHeap *h = heap_create(g->n_nodes);
    heap_update(h, src, use_heuristic ? heuristic(g, src, dst, mode) : 0);

    while (h->size > 0) {
        int u = heap_pop(h);
        done[u] = 1;
        visit(r, u);
        if (u == dst) break;

        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            int v = g->edges[a->edge].to;
            double w = edge_weight(g, a->edge, mode);
            r->relaxations++;
            if (done[v] || w >= INF) continue;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                heap_update(h, v, dist[v] + (use_heuristic ? heuristic(g, v, dst, mode) : 0));
            }
        }
    }
    r->pq_ops = h->ops;
    heap_free(h);
}

static void dijkstra_array(const Graph *g, int dst, WeightMode mode, PathResult *r)
{
    for (int step = 0; step < g->n_nodes; step++) {
        int u = -1;
        for (int v = 0; v < g->n_nodes; v++)
            if (!done[v] && dist[v] < INF && (u < 0 || dist[v] < dist[u])) u = v;
        if (u < 0) break;

        done[u] = 1;
        visit(r, u);
        if (u == dst) break;

        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            int v = g->edges[a->edge].to;
            double w = edge_weight(g, a->edge, mode);
            r->relaxations++;
            if (!done[v] && w < INF && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
            }
        }
    }
}

static void bfs(const Graph *g, int src, int dst, PathResult *r)
{
    Queue *q = queue_create(g->n_nodes);
    done[src] = 1;
    queue_push(q, src);

    while (q->size > 0) {
        int u = queue_pop(q);
        visit(r, u);
        if (u == dst) break;

        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            int v = g->edges[a->edge].to;
            r->relaxations++;
            if (done[v] || g->edges[a->edge].closed) continue;
            done[v] = 1;
            prev[v] = u;
            queue_push(q, v);
        }
    }
    queue_free(q);
}

static void bellman_ford(const Graph *g, int src, WeightMode mode, PathResult *r)
{
    done[src] = 1;
    visit(r, src);

    for (int pass = 0; pass < g->n_nodes - 1; pass++) {
        int changed = 0;
        for (int e = 0; e < g->n_edges; e++) {
            int u = g->edges[e].from, v = g->edges[e].to;
            double w = edge_weight(g, e, mode);
            r->relaxations++;
            if (dist[u] >= INF || w >= INF) continue;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                changed = 1;
                if (!done[v]) {
                    done[v] = 1;
                    visit(r, v);
                }
            }
        }
        if (!changed) break;
    }
}

static void build_path(const Graph *g, PathResult *r, int dst)
{
    int len = 0;
    for (int v = dst; v != -1; v = prev[v]) len++;

    int i = len;
    for (int v = dst; v != -1; v = prev[v]) r->path[--i] = v;

    r->found = 1;
    r->path_len = len;
    for (int k = 0; k + 1 < len; k++) {
        int e = find_edge(g, r->path[k], r->path[k + 1]);
        r->distance_m += g->edges[e].length;
        r->time_s += travel_time(g, e);
    }
}

PathResult find_path(const Graph *g, int src, int dst, WeightMode mode, Algorithm algo)
{
    PathResult r;
    memset(&r, 0, sizeof r);
    r.cost = INF;
    r.path = malloc(sizeof(int) * g->n_nodes);
    r.order = malloc(sizeof(int) * g->n_nodes);
    double start = now_us();

    for (int i = 0; i < g->n_nodes; i++) {
        dist[i] = INF;
        prev[i] = -1;
        done[i] = 0;
    }
    dist[src] = 0;

    if (algo == DIJKSTRA) best_first(g, src, dst, mode, 0, &r);
    if (algo == ASTAR) best_first(g, src, dst, mode, 1, &r);
    if (algo == BFS) bfs(g, src, dst, &r);
    if (algo == BELLMAN_FORD) bellman_ford(g, src, mode, &r);
    if (algo == DIJKSTRA_ARRAY) dijkstra_array(g, dst, mode, &r);

    if (algo == BFS && done[dst]) {
        build_path(g, &r, dst);
        r.cost = mode == BY_DISTANCE ? r.distance_m : r.time_s;
    } else if (algo != BFS && dist[dst] < INF) {
        build_path(g, &r, dst);
        r.cost = dist[dst];
    }

    r.runtime_us = now_us() - start;
    return r;
}

void path_free(PathResult *r)
{
    free(r->path);
    free(r->order);
}

void path_print(const Graph *g, const PathResult *r, Algorithm algo)
{
    if (!r->found) {
        printf("  %s: no route found\n", ALGO_NAME[algo]);
        return;
    }
    printf("  Route: ");
    for (int i = 0; i < r->path_len; i++)
        printf("%s%s", g->nodes[r->path[i]].name, i + 1 < r->path_len ? " -> " : "\n");
    printf("  Distance %.2f km | ETA %.1f min | %d intersections | %d nodes explored | %.1f us\n",
           r->distance_m / 1000, r->time_s / 60, r->path_len, r->visited, r->runtime_us);
    printf("  Exploration order:");
    for (int i = 0; i < r->visited; i++) printf(" %d", r->order[i]);
    printf("\n");
}
