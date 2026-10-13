#include "city.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static void build_signals(Simulation *s)
{
    Graph *g = s->g;
    for (int v = 0; v < g->n_nodes; v++) {
        s->signal_of[v] = -1;
        Signal sig;
        memset(&sig, 0, sizeof sig);
        sig.node = v;
        sig.preempt_edge = -1;

        for (int e = 0; e < g->n_edges && sig.n_in < MAX_IN_EDGES; e++) {
            if (g->edges[e].to != v) continue;
            double dx = g->nodes[v].x - g->nodes[g->edges[e].from].x;
            double dy = g->nodes[v].y - g->nodes[g->edges[e].from].y;
            sig.group[sig.n_in] = fabs(dx) >= fabs(dy) ? 0 : 1;
            sig.in_edges[sig.n_in] = e;
            sig.n_in++;
        }
        if (sig.n_in < 3) continue;

        sig.offset = fmod(v * 7.0, 2 * (GREEN_TIME + YELLOW_TIME));
        s->signal_of[v] = s->n_signals;
        s->signals[s->n_signals] = sig;
        s->n_signals++;
    }
}

static Light signal_light(const Simulation *s, int edge)
{
    int idx = s->signal_of[s->g->edges[edge].to];
    if (idx < 0) return NO_LIGHT;
    const Signal *sig = &s->signals[idx];

    if (sig->preempt_edge >= 0) return sig->preempt_edge == edge ? PREEMPT : RED;

    int group = 0;
    for (int i = 0; i < sig->n_in; i++)
        if (sig->in_edges[i] == edge) group = sig->group[i];

    double phase_len = GREEN_TIME + YELLOW_TIME;
    double t = fmod(s->t + sig->offset, 2 * phase_len);
    int phase = t < phase_len ? 0 : 1;
    if (group != phase) return RED;
    if (t - phase * phase_len < GREEN_TIME) return GREEN;
    return YELLOW;
}

static int spawn_vehicle(Simulation *s, int src, int dst, int is_ambulance, double start)
{
    Graph *g = s->g;
    PathResult r = find_path(g, src, dst, BY_TIME, is_ambulance ? ASTAR : DIJKSTRA);
    free(r.order);
    if (!r.found || r.path_len < 2 || s->n_veh == MAX_VEHICLES) {
        free(r.path);
        return -1;
    }

    Vehicle *v = &s->veh[s->n_veh];
    memset(v, 0, sizeof *v);
    v->is_ambulance = is_ambulance;
    v->route = r.path;
    v->route_len = r.path_len;
    v->leg = 0;
    v->edge = find_edge(g, v->route[0], v->route[1]);
    v->pos = g->edges[v->edge].length * start;
    v->state = MOVING;
    v->spawn_t = s->t;
    v->arrive_t = -1;
    g->edges[v->edge].load++;

    if (is_ambulance) s->amb_planned_s = r.time_s;
    return s->n_veh++;
}

static void spawn_random(Simulation *s, double start)
{
    Graph *g = s->g;
    int landmarks[MAP_MAX_NODES], n_land = 0;
    for (int i = 0; i < g->n_nodes; i++)
        if (g->nodes[i].type == LANDMARK) landmarks[n_land++] = i;

    int src = rng_int(0, g->n_nodes - 1), dst;
    do {
        if (n_land && rng_double() < 0.5)
            dst = landmarks[rng_int(0, n_land - 1)];
        else
            dst = rng_int(0, g->n_nodes - 1);
    } while (dst == src);

    spawn_vehicle(s, src, dst, 0, start);
}

static void next_road(Simulation *s, Vehicle *v)
{
    Graph *g = s->g;
    g->edges[v->edge].load--;
    v->leg++;
    v->edge = find_edge(g, v->route[v->leg], v->route[v->leg + 1]);
    v->pos = 0;
    v->state = MOVING;
    g->edges[v->edge].load++;
}

static int has_priority(const Simulation *s, const Vehicle *v)
{
    return v->is_ambulance && s->corridor;
}

static void update_preemption(Simulation *s)
{
    for (int i = 0; i < s->n_signals; i++) s->signals[i].preempt_edge = -1;
    if (!s->corridor || s->ambulance < 0) return;

    Vehicle *a = &s->veh[s->ambulance];
    if (a->state == ARRIVED) return;
    const Edge *e = &s->g->edges[a->edge];
    int idx = s->signal_of[e->to];
    if (idx < 0 || e->length - a->pos > PREEMPT_RANGE) return;
    if (a->leg + 2 == a->route_len) return;

    if (s->last_preempt_edge != a->edge) s->preemptions++;
    s->last_preempt_edge = a->edge;
    s->signals[idx].preempt_edge = a->edge;
}

static void drive(Simulation *s, int i)
{
    Graph *g = s->g;
    Vehicle *v = &s->veh[i];
    if (v->state == ARRIVED) return;
    if (v->state == QUEUED) {
        v->wait_s += DT;
        return;
    }

    Edge *e = &g->edges[v->edge];
    double speed = e->speed / (1 + 0.15 * pow(congestion(g, v->edge), 4));
    if (has_priority(s, v)) speed = e->speed * 1.15;
    if (speed < MIN_SPEED) speed = MIN_SPEED;
    double before = v->pos;
    v->pos += speed * DT;

    int last_road = (v->leg + 2 == v->route_len);
    if (last_road || has_priority(s, v)) {
        if (v->pos < e->length) {
            v->dist_m += speed * DT;
            return;
        }
        v->dist_m += e->length - before;
        if (!last_road) {
            next_road(s, v);
            return;
        }
        v->state = ARRIVED;
        v->arrive_t = s->t;
        e->load--;
        return;
    }

    Queue *q = s->stop_line[v->edge];
    double stop_at = e->length - CAR_SPACING * q->size;
    if (stop_at < 0) stop_at = 0;
    if (v->pos >= stop_at) {
        v->pos = stop_at;
        if (v->pos < before) v->pos = before;
        v->state = QUEUED;
        queue_push(q, i);
        if (q->size > s->max_queue) s->max_queue = q->size;
    }
    v->dist_m += v->pos - before;
}

static void discharge(Simulation *s)
{
    Graph *g = s->g;
    for (int e = 0; e < g->n_edges; e++) {
        Queue *q = s->stop_line[e];
        Light light = signal_light(s, e);
        if (q->size == 0 || light == RED || light == YELLOW) {
            s->credit[e] = 0;
            continue;
        }
        s->credit[e] += SAT_FLOW * g->edges[e].lanes * DT;
        while (q->size > 0 && s->credit[e] >= 1) {
            Vehicle *v = &s->veh[queue_pop(q)];
            v->dist_m += g->edges[e].length - v->pos;
            next_road(s, v);
            s->credit[e] -= 1;
        }
    }
}

static void record_frame(Simulation *s)
{
    Graph *g = s->g;
    Frame *f = &s->frames[s->n_frames];
    f->t = s->t;
    f->n_veh = 0;
    f->queued = 0;
    f->veh = malloc(sizeof(int) * 4 * (s->n_veh + 1));
    f->lights = malloc(g->n_edges + 1);
    f->cong = malloc(g->n_edges);

    for (int i = 0; i < s->n_veh; i++) {
        Vehicle *v = &s->veh[i];
        if (v->state == ARRIVED) continue;
        double len = g->edges[v->edge].length;
        double shown = v->pos;
        if (v->state == QUEUED) {
            int slot = queue_find(s->stop_line[v->edge], i);
            shown = len - CAR_SPACING * (slot < 0 ? 0 : slot);
            if (shown < 0) shown = 0;
            f->queued++;
        }
        int *out = &f->veh[4 * f->n_veh];
        out[0] = i;
        out[1] = v->edge;
        out[2] = (int)(1000 * shown / len);
        out[3] = (v->state == QUEUED ? 1 : 0) | (v->is_ambulance ? 2 : 0);
        f->n_veh++;
    }

    double sum = 0;
    for (int e = 0; e < g->n_edges; e++) {
        double c = congestion(g, e);
        int pct = (int)(c * 100 + 0.5);
        f->cong[e] = (unsigned char)(pct > 250 ? 250 : pct);
        f->lights[e] = (char)('0' + signal_light(s, e));
        sum += c;
    }
    f->lights[g->n_edges] = '\0';
    f->avg_cong = sum / g->n_edges;
    s->n_frames++;
}

Simulation *sim_create(Graph *g, int corridor)
{
    Simulation *s = calloc(1, sizeof(Simulation));
    s->corridor = corridor;
    s->g = g;
    s->ambulance = -1;
    s->last_preempt_edge = -1;
    s->signals = calloc(g->n_nodes, sizeof(Signal));
    s->signal_of = malloc(sizeof(int) * g->n_nodes);
    s->credit = calloc(g->n_edges, sizeof(double));
    s->stop_line = malloc(sizeof(Queue *) * g->n_edges);
    for (int e = 0; e < g->n_edges; e++) s->stop_line[e] = queue_create(16);
    reset_load(g);
    build_signals(s);
    return s;
}

void sim_run(Simulation *s)
{
    double to_spawn = 0;
    rng_seed(SIM_SEED);
    for (int i = 0; i < START_VEHICLES; i++) spawn_random(s, rng_double() * 0.8);
    record_frame(s);

    for (int tick = 1; tick <= SIM_TICKS; tick++) {
        s->t = tick * DT;

        if (s->ambulance < 0 && s->t >= AMB_START) s->ambulance = spawn_vehicle(s, AMB_FROM, AMB_TO, 1, 0);
        for (to_spawn += SPAWN_RATE * DT; to_spawn >= 1; to_spawn -= 1) spawn_random(s, 0);

        update_preemption(s);
        for (int i = 0; i < s->n_veh; i++) drive(s, i);
        discharge(s);

        if (tick % RECORD_EVERY == 0) record_frame(s);
    }
}

void sim_free(Simulation *s)
{
    for (int i = 0; i < s->n_veh; i++) free(s->veh[i].route);
    for (int e = 0; e < s->g->n_edges; e++) queue_free(s->stop_line[e]);
    for (int f = 0; f < s->n_frames; f++) {
        free(s->frames[f].veh);
        free(s->frames[f].lights);
        free(s->frames[f].cong);
    }
    free(s->stop_line);
    free(s->credit);
    free(s->signals);
    free(s->signal_of);
    free(s);
}

SimStats sim_stats(const Simulation *s)
{
    SimStats st;
    memset(&st, 0, sizeof st);
    st.corridor = s->corridor;
    st.preemptions = s->preemptions;
    st.max_queue = s->max_queue;
    st.amb_planned_s = s->amb_planned_s;

    double trip = 0, wait = 0;
    for (int i = 0; i < s->n_veh; i++) {
        const Vehicle *v = &s->veh[i];
        if (v->is_ambulance) {
            st.amb_arrived = v->state == ARRIVED;
            st.amb_time_s = (st.amb_arrived ? v->arrive_t : s->t) - v->spawn_t;
            st.amb_wait_s = v->wait_s;
            st.amb_dist_m = v->dist_m;
            continue;
        }
        st.vehicles_total++;
        if (v->state == ARRIVED) {
            st.vehicles_done++;
            trip += v->arrive_t - v->spawn_t;
        }
        wait += v->wait_s;
    }
    st.avg_trip_s = st.vehicles_done ? trip / st.vehicles_done : 0;
    st.avg_wait_s = st.vehicles_total ? wait / st.vehicles_total : 0;

    double sum = 0;
    for (int f = 0; f < s->n_frames; f++) {
        sum += s->frames[f].avg_cong;
        for (int e = 0; e < s->g->n_edges; e++)
            if (s->frames[f].cong[e] / 100.0 > st.peak_congestion) st.peak_congestion = s->frames[f].cong[e] / 100.0;
    }
    st.avg_congestion = s->n_frames ? sum / s->n_frames : 0;
    return st;
}

void sim_print_stats(const Simulation *s, const SimStats *st)
{
    const Graph *g = s->g;
    printf("\n  === Simulation %s green corridor ===\n", st->corridor ? "WITH" : "WITHOUT");
    if (s->ambulance >= 0) {
        const Vehicle *a = &s->veh[s->ambulance];
        printf("  Ambulance route (A*): ");
        for (int i = 0; i < a->route_len; i++)
            printf("%s%s", g->nodes[a->route[i]].name, i + 1 < a->route_len ? " -> " : "\n");
    }
    printf("  Ambulance: %s in %.0f s, stopped %.0f s, %.2f km, %d signals pre-empted\n",
           st->amb_arrived ? "arrived" : "DID NOT ARRIVE", st->amb_time_s, st->amb_wait_s, st->amb_dist_m / 1000,
           st->preemptions);
    printf("  Vehicles: %d spawned, %d arrived, avg trip %.0f s, avg stopped %.0f s\n", st->vehicles_total,
           st->vehicles_done, st->avg_trip_s, st->avg_wait_s);
    printf("  Congestion: avg %.0f%%, peak %.0f%%, longest queue %d vehicles\n", st->avg_congestion * 100,
           st->peak_congestion * 100, st->max_queue);
}
