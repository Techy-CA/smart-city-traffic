#include "graph.h"
#include "util.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Graph *graph_create(int node_hint, int edge_hint)
{
    Graph *g     = xcalloc(1, sizeof(Graph));
    g->cap_nodes = node_hint > 0 ? node_hint : 16;
    g->cap_edges = edge_hint > 0 ? edge_hint : 32;
    g->nodes     = xmalloc(sizeof(Node) * g->cap_nodes);
    g->edges     = xmalloc(sizeof(Edge) * g->cap_edges);
    g->adj       = xcalloc(g->cap_nodes, sizeof(AdjNode *));
    return g;
}

void graph_free(Graph *g)
{
    if (!g) return;
    for (int u = 0; u < g->n_nodes; u++) {
        AdjNode *a = g->adj[u];
        while (a) {
            AdjNode *next = a->next;
            free(a);
            a = next;
        }
    }
    free(g->adj);
    free(g->nodes);
    free(g->edges);
    free(g);
}

int graph_add_node(Graph *g, const char *name, double x, double y, NodeType type)
{
    if (g->n_nodes == g->cap_nodes) {
        int old      = g->cap_nodes;
        g->cap_nodes *= 2;
        g->nodes     = xrealloc(g->nodes, sizeof(Node) * g->cap_nodes);
        g->adj       = xrealloc(g->adj, sizeof(AdjNode *) * g->cap_nodes);
        memset(g->adj + old, 0, sizeof(AdjNode *) * (g->cap_nodes - old));
    }
    Node *n = &g->nodes[g->n_nodes];
    n->id   = g->n_nodes;
    snprintf(n->name, NAME_LEN, "%s", name);
    n->x    = x;
    n->y    = y;
    n->type = type;
    return g->n_nodes++;
}

int graph_add_edge(Graph *g, int u, int v, int lanes, double speed_kmh)
{
    if (g->n_edges == g->cap_edges) {
        g->cap_edges *= 2;
        g->edges = xrealloc(g->edges, sizeof(Edge) * g->cap_edges);
    }
    Edge *e     = &g->edges[g->n_edges];
    e->id       = g->n_edges;
    e->from     = u;
    e->to       = v;
    e->length   = euclid(g->nodes[u].x, g->nodes[u].y, g->nodes[v].x, g->nodes[v].y) * METERS_PER_UNIT;
    e->speed    = speed_kmh / 3.6;
    e->lanes    = lanes;
    e->capacity = lanes * VEHICLES_PER_LANE;
    e->load     = 0;
    e->closed   = 0;
    e->reverse  = -1;
    if (e->speed > g->max_speed) g->max_speed = e->speed;

    /* insert at head of u's adjacency list: O(1) */
    AdjNode *a = xmalloc(sizeof(AdjNode));
    a->edge    = e->id;
    a->next    = g->adj[u];
    g->adj[u]  = a;
    return g->n_edges++;
}

int graph_add_road(Graph *g, int u, int v, int lanes, double speed_kmh)
{
    int a = graph_add_edge(g, u, v, lanes, speed_kmh);
    int b = graph_add_edge(g, v, u, lanes, speed_kmh);
    g->edges[a].reverse = b;
    g->edges[b].reverse = a;
    return a;
}

int graph_find_edge(const Graph *g, int u, int v)
{
    for (AdjNode *a = g->adj[u]; a; a = a->next)
        if (g->edges[a->edge].to == v) return a->edge;
    return -1;
}

int graph_find_node_by_name(const Graph *g, const char *name)
{
    for (int i = 0; i < g->n_nodes; i++) {
        const char *p = g->nodes[i].name, *q = name;
        while (*p && *q && tolower((unsigned char)*p) == tolower((unsigned char)*q)) p++, q++;
        if (!*p && !*q) return i;
    }
    return -1;
}

/*
 * Map file format (lines starting with '#' are comments):
 *   NODE <id> <x> <y> <type:junction|hospital|landmark> <Name_with_underscores>
 *   ROAD <u> <v> <lanes> <speed_kmh>
 */
Graph *graph_load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return NULL;

    Graph *g = graph_create(32, 128);
    char line[256];
    int lineno = 0;
    while (fgets(line, sizeof line, f)) {
        lineno++;
        char kind[16];
        if (sscanf(line, "%15s", kind) != 1 || kind[0] == '#') continue;

        if (strcmp(kind, "NODE") == 0) {
            int id;
            double x, y;
            char type[16], name[NAME_LEN];
            if (sscanf(line, "NODE %d %lf %lf %15s %31s", &id, &x, &y, type, name) != 5 || id != g->n_nodes) {
                fprintf(stderr, "%s:%d: bad NODE line (ids must be 0,1,2,... in order)\n", path, lineno);
                graph_free(g);
                fclose(f);
                return NULL;
            }
            for (char *c = name; *c; c++) if (*c == '_') *c = ' ';
            NodeType t = strcmp(type, "hospital") == 0 ? NODE_HOSPITAL
                       : strcmp(type, "landmark") == 0 ? NODE_LANDMARK : NODE_JUNCTION;
            graph_add_node(g, name, x, y, t);
        } else if (strcmp(kind, "ROAD") == 0) {
            int u, v, lanes;
            double kmh;
            if (sscanf(line, "ROAD %d %d %d %lf", &u, &v, &lanes, &kmh) != 4 ||
                u < 0 || v < 0 || u >= g->n_nodes || v >= g->n_nodes || u == v || lanes < 1) {
                fprintf(stderr, "%s:%d: bad ROAD line\n", path, lineno);
                graph_free(g);
                fclose(f);
                return NULL;
            }
            graph_add_road(g, u, v, lanes, kmh);
        } else {
            fprintf(stderr, "%s:%d: unknown record '%s'\n", path, lineno, kind);
        }
    }
    fclose(f);
    return g;
}

/* rows x cols grid with slightly jittered coordinates and random speed limits */
Graph *graph_create_grid(int rows, int cols, unsigned seed)
{
    rng_seed(seed);
    Graph *g = graph_create(rows * cols, rows * cols * 4);
    char name[NAME_LEN];
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
            snprintf(name, sizeof name, "J%d_%d", r, c);
            graph_add_node(g, name, c * 100.0 + rng_int(-15, 15), r * 100.0 + rng_int(-15, 15), NODE_JUNCTION);
        }
    static const double speeds[] = {30, 40, 50, 60};
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
            int u = r * cols + c;
            if (c + 1 < cols) graph_add_road(g, u, u + 1, rng_int(1, 3), speeds[rng_int(0, 3)]);
            if (r + 1 < rows) graph_add_road(g, u, u + cols, rng_int(1, 3), speeds[rng_int(0, 3)]);
        }
    return g;
}

double edge_congestion(const Graph *g, int e)
{
    const Edge *ed = &g->edges[e];
    return ed->capacity ? (double)ed->load / ed->capacity : 0.0;
}

double edge_free_time(const Graph *g, int e) { return g->edges[e].length / g->edges[e].speed; }

/*
 * BPR (Bureau of Public Roads) volume-delay function, the standard traffic
 * engineering model:   t = t0 * (1 + 0.15 * (v/c)^4)
 */
double edge_travel_time(const Graph *g, int e)
{
    double vc = edge_congestion(g, e);
    return edge_free_time(g, e) * (1.0 + 0.15 * pow(vc, 4));
}

double edge_weight(const Graph *g, int e, WeightMode mode)
{
    if (g->edges[e].closed) return INF;
    return mode == WEIGHT_DISTANCE ? g->edges[e].length : edge_travel_time(g, e);
}

void graph_reset_load(Graph *g)
{
    for (int i = 0; i < g->n_edges; i++) g->edges[i].load = 0;
}

void graph_print(const Graph *g)
{
    static const char *types[] = {"junction", "HOSPITAL", "landmark"};
    printf("\n  City map: %d intersections, %d one-way road segments (%d two-way roads)\n\n",
           g->n_nodes, g->n_edges, g->n_edges / 2);
    printf("  %-3s %-18s %-9s  %s\n", "ID", "Name", "Type", "Connected to (distance m, lanes, km/h)");
    printf("  --- ------------------ ---------  --------------------------------------\n");
    for (int u = 0; u < g->n_nodes; u++) {
        printf("  %-3d %-18s %-9s ", u, g->nodes[u].name, types[g->nodes[u].type]);
        int first = 1;
        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            const Edge *e = &g->edges[a->edge];
            printf("%s%d(%.0f,%d,%.0f%s)", first ? " " : ", ", e->to, e->length, e->lanes,
                   e->speed * 3.6, e->closed ? ",CLOSED" : "");
            first = 0;
        }
        printf("\n");
    }
}
