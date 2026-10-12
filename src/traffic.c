#include "traffic.h"
#include "algorithms.h"
#include "util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAR_SPACING_M      9.0   /* car length + gap in a stopped queue        */
#define SAT_FLOW_PER_LANE  0.5   /* vehicles / second / lane leaving on green  */
#define PREEMPT_RANGE_M  300.0   /* ambulance requests green this far out      */
#define MIN_SPEED          1.5   /* m/s, crawling in a jam                     */

SimConfig sim_default_config(int corridor)
{
    SimConfig c;
    c.corridor      = corridor;
    c.n_initial     = 120;
    c.spawn_rate    = 0.55;
    c.duration_s    = 480;
    c.dt            = 0.5;
    c.record_every  = 4;     /* one frame every 2 simulated seconds */
    c.ambulance_src = 13;    /* City Hospital  */
    c.ambulance_dst = 4;     /* Stadium        */
    c.ambulance_t   = 40;
    c.seed          = 2026;
    return c;
}

/* ------------------------------------------------------------ signals */

static void build_signals(Simulation *s)
{
    Graph *g     = s->g;
    s->signal_of = xmalloc(sizeof(int) * g->n_nodes);
    s->signals   = xcalloc(g->n_nodes, sizeof(Signal));
    for (int v = 0; v < g->n_nodes; v++) s->signal_of[v] = -1;

    for (int v = 0; v < g->n_nodes; v++) {
        Signal sig;
        memset(&sig, 0, sizeof sig);
        sig.node         = v;
        sig.preempt_edge = -1;
        for (int e = 0; e < g->n_edges && sig.n_in < MAX_IN_EDGES; e++) {
            if (g->edges[e].to != v) continue;
            const Node *a = &g->nodes[g->edges[e].from];
            double dx = g->nodes[v].x - a->x, dy = g->nodes[v].y - a->y;
            sig.group[sig.n_in]      = fabs(dx) >= fabs(dy) ? 0 : 1;
            sig.in_edges[sig.n_in++] = e;
        }
        if (sig.n_in < 3) continue; /* plain bends / T-less corners: no light */
        sig.offset = fmod(v * 7.0, 2 * (SIGNAL_GREEN_S + SIGNAL_YELLOW_S));
        s->signal_of[v]               = s->n_signals;
        s->signals[s->n_signals++]    = sig;
    }
}

Light signal_light(const Simulation *s, int edge)
{
    int idx = s->signal_of[s->g->edges[edge].to];
    if (idx < 0) return LIGHT_NONE;
    const Signal *sig = &s->signals[idx];

    if (sig->preempt_edge >= 0) return sig->preempt_edge == edge ? LIGHT_PREEMPT : LIGHT_RED;

    int group = 0;
    for (int i = 0; i < sig->n_in; i++)
        if (sig->in_edges[i] == edge) group = sig->group[i];

    double phase_len = SIGNAL_GREEN_S + SIGNAL_YELLOW_S;
    double tt        = fmod(s->t + sig->offset, 2 * phase_len);
    int    phase     = tt < phase_len ? 0 : 1;
    double within    = tt - phase * phase_len;
    if (group != phase) return LIGHT_RED;
    return within < SIGNAL_GREEN_S ? LIGHT_GREEN : LIGHT_YELLOW;
}

/* ------------------------------------------------------------ vehicles */

static int spawn_vehicle(Simulation *s, int src, int dst, int is_ambulance, double start_pos_frac)
{
    Graph *g = s->g;
    /* Ambulance plans with A* on live travel times; cars with Dijkstra. */
    PathResult r = find_path(g, src, dst, WEIGHT_TIME, is_ambulance ? ALGO_ASTAR : ALGO_DIJKSTRA, 0);
    if (!r.found || r.path_len < 2) {
        path_result_free(&r);
        return -1;
    }
    if (s->n_veh == s->cap_veh) {
        s->cap_veh *= 2;
        s->veh = xrealloc(s->veh, sizeof(Vehicle) * s->cap_veh);
    }
    Vehicle *v      = &s->veh[s->n_veh];
    memset(v, 0, sizeof *v);
    v->id           = s->n_veh;
    v->is_ambulance = is_ambulance;
    v->route        = xmalloc(sizeof(int) * r.path_len);
    memcpy(v->route, r.path, sizeof(int) * r.path_len);
    v->route_len    = r.path_len;
    v->leg          = 0;
    v->edge         = graph_find_edge(g, v->route[0], v->route[1]);
    v->pos          = g->edges[v->edge].length * start_pos_frac;
    v->state        = VEH_MOVING;
    v->spawn_t      = s->t;
    v->arrive_t     = -1;
    g->edges[v->edge].load++;

    if (is_ambulance) {
        s->amb_route     = xmalloc(sizeof(int) * r.path_len);
        memcpy(s->amb_route, r.path, sizeof(int) * r.path_len);
        s->amb_route_len = r.path_len;
        s->amb_planned_s = r.time_s;
    }
    path_result_free(&r);
    return s->n_veh++;
}

static void spawn_random(Simulation *s, double start_pos_frac)
{
    Graph *g = s->g;
    int    src = rng_int(0, g->n_nodes - 1), dst;
    /* half of all trips head to a landmark (offices, malls, the station) --
     * that is what creates rush-hour hot spots in the centre */
    int landmarks[64], n_land = 0;
    for (int i = 0; i < g->n_nodes && n_land < 64; i++)
        if (g->nodes[i].type == NODE_LANDMARK) landmarks[n_land++] = i;
    do {
        dst = (n_land && rng_double() < 0.5) ? landmarks[rng_int(0, n_land - 1)] : rng_int(0, g->n_nodes - 1);
    } while (dst == src);
    spawn_vehicle(s, src, dst, 0, start_pos_frac);
}

/* Move a vehicle from the end of its current edge onto the next one. */
static void advance_leg(Simulation *s, Vehicle *v)
{
    Graph *g = s->g;
    g->edges[v->edge].load--;
    v->leg++;
    v->edge  = graph_find_edge(g, v->route[v->leg], v->route[v->leg + 1]);
    v->pos   = 0;
    v->state = VEH_MOVING;
    g->edges[v->edge].load++;
}

static int is_priority(const Simulation *s, const Vehicle *v) { return v->is_ambulance && s->cfg.corridor; }

static void update_preemption(Simulation *s)
{
    for (int i = 0; i < s->n_signals; i++) s->signals[i].preempt_edge = -1;
    if (!s->cfg.corridor || s->ambulance < 0) return;

    Vehicle *a = &s->veh[s->ambulance];
    if (a->state == VEH_ARRIVED) return;
    const Edge *e = &s->g->edges[a->edge];
    int idx       = s->signal_of[e->to];
    if (idx < 0 || e->length - a->pos > PREEMPT_RANGE_M) return;
    if (a->leg + 1 == a->route_len - 1) return; /* destination node: no need */
    if (s->last_preempt_edge != a->edge) s->preemptions++; /* count each signal once */
    s->last_preempt_edge         = a->edge;
    s->signals[idx].preempt_edge = a->edge;
}

/* ------------------------------------------------------------ recording */

static void record_frame(Simulation *s)
{
    Graph *g = s->g;
    if (s->n_frames == s->cap_frames) {
        s->cap_frames      = s->cap_frames ? s->cap_frames * 2 : 256;
        s->frames          = xrealloc(s->frames, sizeof(Frame) * s->cap_frames);
        s->avg_cong_series = xrealloc(s->avg_cong_series, sizeof(double) * s->cap_frames);
        s->active_series   = xrealloc(s->active_series, sizeof(int) * s->cap_frames);
        s->queued_series   = xrealloc(s->queued_series, sizeof(int) * s->cap_frames);
    }
    Frame *f  = &s->frames[s->n_frames];
    f->t      = s->t;
    f->veh    = xmalloc(sizeof(int) * 4 * (s->n_veh + 1));
    f->lights = xmalloc(g->n_edges + 1);
    f->cong   = xmalloc(g->n_edges);
    f->n_veh  = 0;

    int queued = 0;
    for (int i = 0; i < s->n_veh; i++) {
        Vehicle *v = &s->veh[i];
        if (v->state == VEH_ARRIVED) continue;
        double len  = g->edges[v->edge].length;
        double disp = v->pos;
        if (v->state == VEH_QUEUED) {
            int slot = queue_index_of(s->stop_line[v->edge], v->id);
            disp     = len - CAR_SPACING_M * (slot < 0 ? 0 : slot);
            if (disp < 0) disp = 0;
            queued++;
        }
        int *o = &f->veh[4 * f->n_veh++];
        o[0]   = v->id;
        o[1]   = v->edge;
        o[2]   = (int)(1000 * disp / len);
        o[3]   = (v->state == VEH_QUEUED ? 1 : 0) | (v->is_ambulance ? 2 : 0);
    }

    double sum = 0;
    for (int e = 0; e < g->n_edges; e++) {
        double c     = edge_congestion(g, e);
        int    pct   = (int)(c * 100 + 0.5);
        f->cong[e]   = (unsigned char)(pct > 250 ? 250 : pct);
        f->lights[e] = (char)('0' + signal_light(s, e));
        sum += c;
    }
    f->lights[g->n_edges] = '\0';

    s->avg_cong_series[s->n_frames] = sum / g->n_edges;
    s->active_series[s->n_frames]   = f->n_veh;
    s->queued_series[s->n_frames]   = queued;
    s->n_frames++;
}

/* ------------------------------------------------------------ main loop */

Simulation *sim_create(Graph *g, SimConfig cfg)
{
    Simulation *s = xcalloc(1, sizeof(Simulation));
    s->cfg        = cfg;
    s->g          = g;
    s->ambulance  = -1;
    s->last_preempt_edge = -1;
    s->cap_veh    = 256;
    s->veh        = xmalloc(sizeof(Vehicle) * s->cap_veh);
    s->stop_line  = xmalloc(sizeof(Queue *) * g->n_edges);
    for (int e = 0; e < g->n_edges; e++) s->stop_line[e] = queue_create(16);
    graph_reset_load(g);
    build_signals(s);
    return s;
}

void sim_run(Simulation *s)
{
    Graph   *g      = s->g;
    double  *credit = xcalloc(g->n_edges, sizeof(double)); /* discharge budget per stop line */
    double   spawn_acc = 0;
    int      ticks     = (int)(s->cfg.duration_s / s->cfg.dt);
    double   dt        = s->cfg.dt;

    rng_seed(s->cfg.seed);
    for (int i = 0; i < s->cfg.n_initial; i++) spawn_random(s, rng_double() * 0.8);
    record_frame(s);

    for (int tick = 1; tick <= ticks; tick++) {
        s->t = tick * dt;

        /* 1. dispatch + new demand */
        if (s->ambulance < 0 && s->t >= s->cfg.ambulance_t)
            s->ambulance = spawn_vehicle(s, s->cfg.ambulance_src, s->cfg.ambulance_dst, 1, 0);
        for (spawn_acc += s->cfg.spawn_rate * dt; spawn_acc >= 1; spawn_acc -= 1) spawn_random(s, 0);

        /* 2. green corridor: ambulance pre-empts the next signal */
        update_preemption(s);

        /* 3. drive: BPR speed on each road, stop at the back of the queue */
        for (int i = 0; i < s->n_veh; i++) {
            Vehicle *v = &s->veh[i];
            if (v->state == VEH_ARRIVED) continue;
            if (v->state == VEH_QUEUED) {
                v->wait_s += dt;
                continue;
            }
            const Edge *e   = &g->edges[v->edge];
            double      vc  = edge_congestion(g, v->edge);
            double      spd = is_priority(s, v) ? e->speed * 1.15        /* traffic pulls over */
                                                : e->speed / (1 + 0.15 * pow(vc, 4));
            if (spd < MIN_SPEED) spd = MIN_SPEED;
            double before = v->pos;
            v->pos += spd * dt;

            int last_leg = (v->leg + 2 == v->route_len);
            if (last_leg) {
                if (v->pos >= e->length) {
                    v->dist_m += e->length - before;
                    v->state    = VEH_ARRIVED;
                    v->arrive_t = s->t;
                    g->edges[v->edge].load--;
                } else {
                    v->dist_m += spd * dt;
                }
                continue;
            }

            if (is_priority(s, v)) {
                if (v->pos >= e->length) {
                    v->dist_m += e->length - before;
                    advance_leg(s, v);
                } else {
                    v->dist_m += spd * dt;
                }
                continue;
            }

            double limit = e->length - CAR_SPACING_M * s->stop_line[v->edge]->size;
            if (limit < 0) limit = 0;
            if (v->pos >= limit) {
                v->pos = limit;
                if (v->pos < before) v->pos = before; /* never roll backwards */
                v->state = VEH_QUEUED;
                queue_push(s->stop_line[v->edge], v->id);
                int qs = s->stop_line[v->edge]->size;
                if (qs > s->max_queue) s->max_queue = qs;
            }
            v->dist_m += v->pos - before;
        }

        /* 4. stop lines discharge FIFO at saturation flow while the light allows */
        for (int e = 0; e < g->n_edges; e++) {
            Queue *q = s->stop_line[e];
            if (!q->size) {
                credit[e] = 0;
                continue;
            }
            Light l = signal_light(s, e);
            if (l == LIGHT_RED || l == LIGHT_YELLOW) {
                credit[e] = 0;
                continue;
            }
            credit[e] += SAT_FLOW_PER_LANE * g->edges[e].lanes * dt;
            while (q->size && credit[e] >= 1) {
                Vehicle *v = &s->veh[queue_pop(q)];
                v->dist_m += g->edges[e].length - v->pos;
                advance_leg(s, v);
                credit[e] -= 1;
            }
        }

        if (tick % s->cfg.record_every == 0) record_frame(s);
    }
    free(credit);
}

void sim_free(Simulation *s)
{
    if (!s) return;
    for (int i = 0; i < s->n_veh; i++) free(s->veh[i].route);
    for (int e = 0; e < s->g->n_edges; e++) queue_free(s->stop_line[e]);
    for (int f = 0; f < s->n_frames; f++) {
        free(s->frames[f].veh);
        free(s->frames[f].lights);
        free(s->frames[f].cong);
    }
    free(s->frames);
    free(s->avg_cong_series);
    free(s->active_series);
    free(s->queued_series);
    free(s->stop_line);
    free(s->veh);
    free(s->signals);
    free(s->signal_of);
    free(s->amb_route);
    free(s);
}

SimStats sim_stats(const Simulation *s)
{
    SimStats st;
    memset(&st, 0, sizeof st);
    st.corridor     = s->cfg.corridor;
    st.preemptions  = s->preemptions;
    st.max_queue    = s->max_queue;
    st.amb_planned_s = s->amb_planned_s;

    double trip = 0, wait = 0;
    for (int i = 0; i < s->n_veh; i++) {
        const Vehicle *v = &s->veh[i];
        if (v->is_ambulance) {
            st.amb_arrived = v->state == VEH_ARRIVED;
            st.amb_time_s  = (st.amb_arrived ? v->arrive_t : s->t) - v->spawn_t;
            st.amb_wait_s  = v->wait_s;
            st.amb_dist_m  = v->dist_m;
            continue;
        }
        st.vehicles_total++;
        if (v->state == VEH_ARRIVED) {
            st.vehicles_done++;
            trip += v->arrive_t - v->spawn_t;
        }
        wait += v->wait_s;
    }
    st.avg_trip_s = st.vehicles_done ? trip / st.vehicles_done : 0;
    st.avg_wait_s = st.vehicles_total ? wait / st.vehicles_total : 0;

    double sum = 0;
    for (int f = 0; f < s->n_frames; f++) {
        sum += s->avg_cong_series[f];
        for (int e = 0; e < s->g->n_edges; e++)
            if (s->frames[f].cong[e] / 100.0 > st.peak_congestion) st.peak_congestion = s->frames[f].cong[e] / 100.0;
    }
    st.avg_congestion = s->n_frames ? sum / s->n_frames : 0;
    return st;
}

void sim_print_stats(const Graph *g, const Simulation *s, const SimStats *st)
{
    printf("\n  === Simulation: %s ===\n", st->corridor ? "WITH green corridor" : "WITHOUT green corridor");
    printf("  Ambulance %s -> %s, dispatched at t=%.0fs\n", g->nodes[s->cfg.ambulance_src].name,
           g->nodes[s->cfg.ambulance_dst].name, s->cfg.ambulance_t);
    if (s->amb_route_len) {
        printf("  A* route: ");
        for (int i = 0; i < s->amb_route_len; i++)
            printf("%s%s", g->nodes[s->amb_route[i]].name, i + 1 < s->amb_route_len ? " -> " : "\n");
    }
    printf("  Ambulance: %s in %.0f s (A* predicted %.0f s), stopped %.0f s, %.2f km, %d signals pre-empted\n",
           st->amb_arrived ? "arrived" : "DID NOT ARRIVE", st->amb_time_s, st->amb_planned_s, st->amb_wait_s,
           st->amb_dist_m / 1000, st->preemptions);
    printf("  Vehicles: %d spawned, %d completed, avg trip %.0f s, avg stopped %.0f s\n", st->vehicles_total,
           st->vehicles_done, st->avg_trip_s, st->avg_wait_s);
    printf("  Congestion: avg %.0f%%, peak %.0f%% of capacity, longest queue %d vehicles\n",
           st->avg_congestion * 100, st->peak_congestion * 100, st->max_queue);
}
