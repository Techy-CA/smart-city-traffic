#include "city.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

Graph *graph_create(int max_nodes, int max_edges)
{
    Graph *g = calloc(1, sizeof(Graph));
    g->max_nodes = max_nodes;
    g->max_edges = max_edges;
    g->nodes = malloc(sizeof(Node) * max_nodes);
    g->edges = malloc(sizeof(Edge) * max_edges);
    g->adj = calloc(max_nodes, sizeof(AdjNode *));
    return g;
}

void graph_free(Graph *g)
{
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

int add_node(Graph *g, const char *name, double x, double y, NodeType type)
{
    Node *n = &g->nodes[g->n_nodes];
    snprintf(n->name, NAME_LEN, "%s", name);
    n->x = x;
    n->y = y;
    n->type = type;
    return g->n_nodes++;
}

static int add_edge(Graph *g, int u, int v, int lanes, double kmh)
{
    Edge *e = &g->edges[g->n_edges];
    e->from = u;
    e->to = v;
    e->length = euclid(g->nodes[u].x, g->nodes[u].y, g->nodes[v].x, g->nodes[v].y) * METERS_PER_UNIT;
    e->speed = kmh / 3.6;
    e->lanes = lanes;
    e->capacity = lanes * VEHICLES_PER_LANE;
    e->load = 0;
    e->closed = 0;
    if (e->speed > g->max_speed) g->max_speed = e->speed;

    AdjNode *a = malloc(sizeof(AdjNode));
    a->edge = g->n_edges;
    a->next = g->adj[u];
    g->adj[u] = a;
    return g->n_edges++;
}

void add_road(Graph *g, int u, int v, int lanes, double kmh)
{
    int a = add_edge(g, u, v, lanes, kmh);
    int b = add_edge(g, v, u, lanes, kmh);
    g->edges[a].reverse = b;
    g->edges[b].reverse = a;
}

int find_edge(const Graph *g, int u, int v)
{
    for (AdjNode *a = g->adj[u]; a; a = a->next)
        if (g->edges[a->edge].to == v) return a->edge;
    return -1;
}

Graph *graph_load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return NULL;

    Graph *g = graph_create(MAP_MAX_NODES, MAP_MAX_EDGES);
    char line[256], type[16], name[NAME_LEN];
    int id, u, v, lanes;
    double x, y, kmh;

    while (fgets(line, sizeof line, f)) {
        if (sscanf(line, "NODE %d %lf %lf %15s %31s", &id, &x, &y, type, name) == 5) {
            if (g->n_nodes == g->max_nodes) continue;
            for (char *c = name; *c; c++)
                if (*c == '_') *c = ' ';
            NodeType t = JUNCTION;
            if (strcmp(type, "hospital") == 0) t = HOSPITAL;
            if (strcmp(type, "landmark") == 0) t = LANDMARK;
            add_node(g, name, x, y, t);
        } else if (sscanf(line, "ROAD %d %d %d %lf", &u, &v, &lanes, &kmh) == 4) {
            if (u < 0 || v < 0 || u >= g->n_nodes || v >= g->n_nodes || lanes < 1) continue;
            if (g->n_edges + 2 > g->max_edges) continue;
            add_road(g, u, v, lanes, kmh);
        }
    }
    fclose(f);
    return g;
}

Graph *graph_create_grid(int n, unsigned seed)
{
    static const double speeds[] = {30, 40, 50, 60};
    char name[NAME_LEN];
    Graph *g = graph_create(n * n, n * n * 4);
    rng_seed(seed);

    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++) {
            double x = c * 100.0 + rng_int(-15, 15);
            double y = r * 100.0 + rng_int(-15, 15);
            snprintf(name, sizeof name, "J%d_%d", r, c);
            add_node(g, name, x, y, JUNCTION);
        }

    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++) {
            int u = r * n + c;
            if (c + 1 < n) {
                int lanes = rng_int(1, 3);
                add_road(g, u, u + 1, lanes, speeds[rng_int(0, 3)]);
            }
            if (r + 1 < n) {
                int lanes = rng_int(1, 3);
                add_road(g, u, u + n, lanes, speeds[rng_int(0, 3)]);
            }
        }
    return g;
}

double congestion(const Graph *g, int e)
{
    return (double)g->edges[e].load / g->edges[e].capacity;
}

double travel_time(const Graph *g, int e)
{
    double free_time = g->edges[e].length / g->edges[e].speed;
    return free_time * (1.0 + 0.15 * pow(congestion(g, e), 4));
}

double edge_weight(const Graph *g, int e, WeightMode mode)
{
    if (g->edges[e].closed) return INF;
    if (mode == BY_DISTANCE) return g->edges[e].length;
    return travel_time(g, e);
}

void reset_load(Graph *g)
{
    for (int i = 0; i < g->n_edges; i++) g->edges[i].load = 0;
}

void graph_print(const Graph *g)
{
    static const char *types[] = {"junction", "HOSPITAL", "landmark"};
    printf("\n  City map: %d intersections, %d two-way roads\n\n", g->n_nodes, g->n_edges / 2);
    printf("  %-3s %-18s %-9s  %s\n", "ID", "Name", "Type", "Connected to (distance m, lanes, km/h)");
    for (int u = 0; u < g->n_nodes; u++) {
        printf("  %-3d %-18s %-9s ", u, g->nodes[u].name, types[g->nodes[u].type]);
        for (AdjNode *a = g->adj[u]; a; a = a->next) {
            const Edge *e = &g->edges[a->edge];
            printf(" %d(%.0f,%d,%.0f%s)", e->to, e->length, e->lanes, e->speed * 3.6, e->closed ? ",CLOSED" : "");
        }
        printf("\n");
    }
}
