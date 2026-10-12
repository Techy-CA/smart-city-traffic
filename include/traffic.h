#ifndef TRAFFIC_H
#define TRAFFIC_H

#include "graph.h"
#include "pq.h"

/* Light colour of an incoming road at its stop line */
typedef enum { LIGHT_NONE = 0, LIGHT_GREEN = 1, LIGHT_YELLOW = 2, LIGHT_RED = 3, LIGHT_PREEMPT = 4 } Light;

#define SIGNAL_GREEN_S   18.0
#define SIGNAL_YELLOW_S   3.0
#define MAX_IN_EDGES      8

/* Two-phase signal: phase 0 serves the east-west approaches, phase 1 the
 * north-south ones. An ambulance can pre-empt it (green corridor). */
typedef struct {
    int    node;
    int    n_in;
    int    in_edges[MAX_IN_EDGES];
    int    group[MAX_IN_EDGES];    /* 0 = E-W phase, 1 = N-S phase */
    double offset;                 /* phase offset, staggers the city */
    int    preempt_edge;           /* edge forced green by an ambulance, -1 if none */
} Signal;

typedef enum { VEH_MOVING = 0, VEH_QUEUED = 1, VEH_ARRIVED = 2 } VehState;

typedef struct {
    int      id;
    int      is_ambulance;
    int     *route;          /* node ids                 */
    int      route_len;
    int      leg;            /* currently on route[leg] -> route[leg+1] */
    int      edge;
    double   pos;            /* metres from start of edge */
    VehState state;
    double   spawn_t, arrive_t;
    double   wait_s;         /* time spent stopped        */
    double   dist_m;
} Vehicle;

typedef struct {
    int    corridor;          /* 1 = ambulance gets green-wave priority      */
    int    n_initial;         /* vehicles on the road at t = 0              */
    double spawn_rate;        /* new vehicles per second                    */
    double duration_s;
    double dt;
    int    record_every;      /* record a frame every N ticks               */
    int    ambulance_src, ambulance_dst;
    double ambulance_t;       /* dispatch time                              */
    unsigned seed;
} SimConfig;

/* One recorded snapshot -- what the visualiser draws. */
typedef struct {
    double t;
    int    n_veh;
    int   *veh;          /* n_veh * 4 ints: id, edge, progress (0..1000), flags */
    char  *lights;       /* n_edges chars '0'..'4' (Light at the edge's stop line) */
    unsigned char *cong; /* n_edges values: congestion percent, capped at 250 */
} Frame;

typedef struct {
    SimConfig cfg;
    Graph    *g;
    Signal   *signals;
    int       n_signals;
    int      *signal_of;   /* node -> signal index or -1 */
    Queue   **stop_line;   /* per-edge FIFO of vehicles waiting at the light */
    Vehicle  *veh;
    int       n_veh, cap_veh;
    int       ambulance;   /* vehicle id, -1 before dispatch */
    double    t;

    Frame    *frames;
    int       n_frames, cap_frames;

    /* stats */
    int      *amb_route;
    int       amb_route_len;
    double    amb_planned_s;  /* ETA predicted by A* when dispatched */
    int       preemptions;
    int       last_preempt_edge;
    int       max_queue;
    double   *avg_cong_series; /* per frame */
    int      *active_series;   /* vehicles on the road, per frame */
    int      *queued_series;   /* vehicles stopped, per frame */
} Simulation;

typedef struct {
    int    corridor;
    double amb_time_s, amb_planned_s, amb_wait_s, amb_dist_m;
    int    amb_arrived;
    int    vehicles_total, vehicles_done;
    double avg_trip_s, avg_wait_s;
    double avg_congestion, peak_congestion;
    int    max_queue, preemptions;
} SimStats;

SimConfig   sim_default_config(int corridor);
Simulation *sim_create(Graph *g, SimConfig cfg);
void        sim_run(Simulation *s);
void        sim_free(Simulation *s);
SimStats    sim_stats(const Simulation *s);
Light       signal_light(const Simulation *s, int edge);
void        sim_print_stats(const Graph *g, const Simulation *s, const SimStats *st);

#endif
