#ifndef GRAPH_H
#define GRAPH_H

/*
 * City road network as a directed, weighted graph.
 *
 *   - Nodes   = intersections / landmarks (with x,y map coordinates)
 *   - Edges   = one-way road segments. A two-way road is two edges that point
 *               at each other through `reverse`.
 *   - Storage = adjacency list (array of singly linked lists), O(V + E) memory.
 */

#define NAME_LEN          32
#define METERS_PER_UNIT   5.0    /* map coordinate unit -> metres            */
#define VEHICLES_PER_LANE 3      /* scaled-model capacity of one lane        */

typedef enum { NODE_JUNCTION = 0, NODE_HOSPITAL = 1, NODE_LANDMARK = 2 } NodeType;

typedef struct {
    int      id;
    char     name[NAME_LEN];
    double   x, y;
    NodeType type;
} Node;

typedef struct {
    int    id;
    int    from, to;
    double length;     /* metres                                   */
    double speed;      /* free-flow speed limit, metres / second   */
    int    lanes;
    int    capacity;   /* vehicles the segment holds before jamming */
    int    load;       /* vehicles currently on the segment         */
    int    closed;     /* 1 = blocked (accident / roadwork)         */
    int    reverse;    /* id of opposite-direction edge, or -1      */
} Edge;

typedef struct AdjNode {
    int             edge;   /* index into Graph.edges */
    struct AdjNode *next;
} AdjNode;

typedef struct {
    int       n_nodes, n_edges;
    int       cap_nodes, cap_edges;
    Node     *nodes;
    Edge     *edges;
    AdjNode **adj;          /* adj[u] -> outgoing edges of u */
    double    max_speed;    /* used by the A* time heuristic */
} Graph;

typedef enum { WEIGHT_DISTANCE = 0, WEIGHT_TIME = 1 } WeightMode;

Graph *graph_create(int node_hint, int edge_hint);
void   graph_free(Graph *g);
int    graph_add_node(Graph *g, const char *name, double x, double y, NodeType type);
int    graph_add_edge(Graph *g, int u, int v, int lanes, double speed_kmh);
int    graph_add_road(Graph *g, int u, int v, int lanes, double speed_kmh); /* two-way */
int    graph_find_edge(const Graph *g, int u, int v);

Graph *graph_load(const char *path);
Graph *graph_create_grid(int rows, int cols, unsigned seed); /* for benchmarks */

double edge_travel_time(const Graph *g, int e);   /* BPR congestion model, seconds */
double edge_free_time(const Graph *g, int e);
double edge_weight(const Graph *g, int e, WeightMode mode);
double edge_congestion(const Graph *g, int e);    /* load / capacity */

void   graph_reset_load(Graph *g);
void   graph_print(const Graph *g);
int    graph_find_node_by_name(const Graph *g, const char *name);

#endif
