#ifndef CITY_H
#define CITY_H

#include <stdio.h>

#define INF 1e18
#define NAME_LEN 32
#define METERS_PER_UNIT 5.0
#define VEHICLES_PER_LANE 3
#define MAX_NODES 10000
#define MAP_MAX_NODES 100
#define MAP_MAX_EDGES 400

typedef enum { JUNCTION, HOSPITAL, LANDMARK } NodeType;
typedef enum { BY_DISTANCE, BY_TIME } WeightMode;
typedef enum { DIJKSTRA, ASTAR, BFS, BELLMAN_FORD, DIJKSTRA_ARRAY, ALGO_COUNT } Algorithm;

typedef struct {
    char name[NAME_LEN];
    double x, y;
    NodeType type;
} Node;

typedef struct {
    int from, to;
    double length;
    double speed;
    int lanes;
    int capacity;
    int load;
    int closed;
    int reverse;
} Edge;

typedef struct AdjNode {
    int edge;
    struct AdjNode *next;
} AdjNode;

typedef struct {
    int n_nodes, n_edges;
    int max_nodes, max_edges;
    Node *nodes;
    Edge *edges;
    AdjNode **adj;
    double max_speed;
} Graph;

typedef struct {
    int *heap;
    int *pos;
    double *key;
    int size;
    long ops;
} MinHeap;

typedef struct {
    int *data;
    int head, size, capacity;
} Queue;

typedef struct {
    int found;
    int *path;
    int path_len;
    double cost;
    double distance_m;
    double time_s;
    int *order;
    int visited;
    long relaxations;
    long pq_ops;
    double runtime_us;
} PathResult;

typedef struct {
    int queries;
    double visited, relax, pq_ops, runtime_us, distance_m;
    double optimal_pct, excess_pct;
} AlgoSummary;

#define BENCH_SIZES 5

typedef struct {
    int V, E;
    double runtime_us[ALGO_COUNT];
    double visited[ALGO_COUNT];
} BenchRow;

#define GREEN_TIME 18.0
#define YELLOW_TIME 3.0
#define MAX_IN_EDGES 8
#define CAR_SPACING 9.0
#define SAT_FLOW 0.5
#define PREEMPT_RANGE 300.0
#define MIN_SPEED 1.5

#define START_VEHICLES 120
#define SPAWN_RATE 0.55
#define SIM_TICKS 960
#define DT 0.5
#define RECORD_EVERY 4
#define MAX_FRAMES (SIM_TICKS / RECORD_EVERY + 1)
#define MAX_VEHICLES 1000
#define AMB_FROM 13
#define AMB_TO 4
#define AMB_START 40.0
#define SIM_SEED 2026

typedef enum { NO_LIGHT, GREEN, YELLOW, RED, PREEMPT } Light;
typedef enum { MOVING, QUEUED, ARRIVED } VehState;

typedef struct {
    int node;
    int n_in;
    int in_edges[MAX_IN_EDGES];
    int group[MAX_IN_EDGES];
    double offset;
    int preempt_edge;
} Signal;

typedef struct {
    int is_ambulance;
    int *route;
    int route_len;
    int leg;
    int edge;
    double pos;
    VehState state;
    double spawn_t, arrive_t;
    double wait_s;
    double dist_m;
} Vehicle;

typedef struct {
    double t;
    int n_veh;
    int queued;
    double avg_cong;
    int *veh;
    char *lights;
    unsigned char *cong;
} Frame;

typedef struct {
    int corridor;
    Graph *g;
    double t;
    Signal *signals;
    int n_signals;
    int *signal_of;
    Queue **stop_line;
    double *credit;
    Vehicle veh[MAX_VEHICLES];
    int n_veh;
    int ambulance;
    double amb_planned_s;
    int preemptions;
    int last_preempt_edge;
    int max_queue;
    Frame frames[MAX_FRAMES];
    int n_frames;
} Simulation;

typedef struct {
    int corridor;
    double amb_time_s, amb_planned_s, amb_wait_s, amb_dist_m;
    int amb_arrived;
    int vehicles_total, vehicles_done;
    double avg_trip_s, avg_wait_s;
    double avg_congestion, peak_congestion;
    int max_queue, preemptions;
} SimStats;

void rng_seed(unsigned seed);
int rng_int(int lo, int hi);
double rng_double(void);
double now_us(void);
double euclid(double x1, double y1, double x2, double y2);

Graph *graph_create(int max_nodes, int max_edges);
void graph_free(Graph *g);
int add_node(Graph *g, const char *name, double x, double y, NodeType type);
void add_road(Graph *g, int u, int v, int lanes, double kmh);
int find_edge(const Graph *g, int u, int v);
Graph *graph_load(const char *path);
Graph *graph_create_grid(int n, unsigned seed);
double congestion(const Graph *g, int e);
double travel_time(const Graph *g, int e);
double edge_weight(const Graph *g, int e, WeightMode mode);
void reset_load(Graph *g);
void graph_print(const Graph *g);

MinHeap *heap_create(int n);
void heap_free(MinHeap *h);
void heap_update(MinHeap *h, int v, double key);
int heap_pop(MinHeap *h);
Queue *queue_create(int capacity);
void queue_free(Queue *q);
void queue_push(Queue *q, int v);
int queue_pop(Queue *q);
int queue_find(const Queue *q, int v);

extern const char *ALGO_NAME[ALGO_COUNT];
extern const char *ALGO_KEY[ALGO_COUNT];
PathResult find_path(const Graph *g, int src, int dst, WeightMode mode, Algorithm algo);
void path_free(PathResult *r);
void path_print(const Graph *g, const PathResult *r, Algorithm algo);

void compare_algorithms(const Graph *g, WeightMode mode, AlgoSummary out[ALGO_COUNT]);
void print_algo_summary(const AlgoSummary s[ALGO_COUNT], WeightMode mode);
void compare_route(const Graph *g, int src, int dst, WeightMode mode);
void benchmark_scaling(BenchRow rows[BENCH_SIZES]);
void print_bench(const BenchRow rows[BENCH_SIZES]);

Simulation *sim_create(Graph *g, int corridor);
void sim_run(Simulation *s);
void sim_free(Simulation *s);
SimStats sim_stats(const Simulation *s);
void sim_print_stats(const Simulation *s, const SimStats *st);

int export_js(const char *path, const Graph *g, Simulation *sims[2], const SimStats stats[2],
              const AlgoSummary dist[ALGO_COUNT], const AlgoSummary time[ALGO_COUNT], const BenchRow bench[BENCH_SIZES]);

#endif
