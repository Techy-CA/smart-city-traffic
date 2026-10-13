#include "city.h"

static void write_ints(FILE *f, const int *a, int n)
{
    fputc('[', f);
    for (int i = 0; i < n; i++) fprintf(f, i ? ",%d" : "%d", a[i]);
    fputc(']', f);
}

static void write_graph(FILE *f, const Graph *g)
{
    static const char *types[] = {"junction", "hospital", "landmark"};
    fprintf(f, "\"nodes\":[");
    for (int i = 0; i < g->n_nodes; i++) {
        const Node *n = &g->nodes[i];
        fprintf(f, "%s{\"id\":%d,\"name\":\"%s\",\"x\":%.1f,\"y\":%.1f,\"type\":\"%s\"}", i ? "," : "", i, n->name, n->x,
                n->y, types[n->type]);
    }
    fprintf(f, "],\n\"edges\":[");
    for (int i = 0; i < g->n_edges; i++) {
        const Edge *e = &g->edges[i];
        fprintf(f, "%s{\"id\":%d,\"from\":%d,\"to\":%d,\"len\":%.1f,\"kmh\":%.0f,\"lanes\":%d,\"cap\":%d,\"rev\":%d}",
                i ? "," : "", i, e->from, e->to, e->length, e->speed * 3.6, e->lanes, e->capacity, e->reverse);
    }
    fprintf(f, "]");
}

static void write_summary(FILE *f, const AlgoSummary *s)
{
    fputc('[', f);
    for (int a = 0; a < ALGO_COUNT; a++)
        fprintf(f,
                "%s{\"key\":\"%s\",\"name\":\"%s\",\"visited\":%.2f,\"relax\":%.2f,\"pqOps\":%.2f,"
                "\"us\":%.3f,\"dist\":%.1f,\"optimal\":%.1f,\"excess\":%.2f}",
                a ? "," : "", ALGO_KEY[a], ALGO_NAME[a], s[a].visited, s[a].relax, s[a].pq_ops, s[a].runtime_us,
                s[a].distance_m, s[a].optimal_pct, s[a].excess_pct);
    fputc(']', f);
}

static void write_bench(FILE *f, const BenchRow *rows)
{
    fprintf(f, "\"bench\":[");
    for (int i = 0; i < BENCH_SIZES; i++) {
        fprintf(f, "%s{\"V\":%d,\"E\":%d,\"us\":{", i ? "," : "", rows[i].V, rows[i].E);
        for (int a = 0; a < ALGO_COUNT; a++) fprintf(f, "%s\"%s\":%.2f", a ? "," : "", ALGO_KEY[a], rows[i].runtime_us[a]);
        fprintf(f, "},\"visited\":{");
        for (int a = 0; a < ALGO_COUNT; a++) fprintf(f, "%s\"%s\":%.1f", a ? "," : "", ALGO_KEY[a], rows[i].visited[a]);
        fprintf(f, "}}");
    }
    fprintf(f, "]");
}

static void write_planner(FILE *f, const Graph *g)
{
    fprintf(f, "\"planner\":{");
    for (int a = DIJKSTRA; a <= BELLMAN_FORD; a++) {
        fprintf(f, "%s\"%s\":[", a ? ",\n" : "", ALGO_KEY[a]);
        for (int s = 0; s < g->n_nodes; s++)
            for (int d = 0; d < g->n_nodes; d++) {
                if (s || d) fputc(',', f);
                if (s == d) {
                    fprintf(f, "null");
                    continue;
                }
                PathResult r = find_path(g, s, d, BY_DISTANCE, a);
                fputc('[', f);
                write_ints(f, r.path, r.found ? r.path_len : 0);
                fputc(',', f);
                write_ints(f, r.order, r.visited);
                fprintf(f, ",%.0f,%d,%ld,%.2f]", r.found ? r.distance_m : -1, r.visited, r.relaxations, r.runtime_us);
                path_free(&r);
            }
        fprintf(f, "]");
    }
    fprintf(f, "}");
}

static void write_sim(FILE *f, const Simulation *s, const SimStats *st)
{
    const Graph *g = s->g;
    fprintf(f, "{\"corridor\":%d,\"dt\":%.2f,", s->corridor, DT * RECORD_EVERY);
    fprintf(f,
            "\"stats\":{\"ambTime\":%.1f,\"ambPlanned\":%.1f,\"ambWait\":%.1f,\"ambDist\":%.0f,\"ambArrived\":%d,"
            "\"vehicles\":%d,\"done\":%d,\"avgTrip\":%.1f,\"avgWait\":%.1f,\"avgCong\":%.3f,\"peakCong\":%.2f,"
            "\"maxQueue\":%d,\"preemptions\":%d},\n",
            st->amb_time_s, st->amb_planned_s, st->amb_wait_s, st->amb_dist_m, st->amb_arrived, st->vehicles_total,
            st->vehicles_done, st->avg_trip_s, st->avg_wait_s, st->avg_congestion, st->peak_congestion, st->max_queue,
            st->preemptions);

    fprintf(f, "\"ambulance\":{\"src\":%d,\"dst\":%d,\"t\":%.1f,\"route\":", AMB_FROM, AMB_TO, AMB_START);
    if (s->ambulance >= 0) write_ints(f, s->veh[s->ambulance].route, s->veh[s->ambulance].route_len);
    else write_ints(f, NULL, 0);

    fprintf(f, "},\n\"series\":{\"cong\":[");
    for (int i = 0; i < s->n_frames; i++) fprintf(f, i ? ",%.3f" : "%.3f", s->frames[i].avg_cong);
    fprintf(f, "],\"active\":[");
    for (int i = 0; i < s->n_frames; i++) fprintf(f, i ? ",%d" : "%d", s->frames[i].n_veh);
    fprintf(f, "],\"queued\":[");
    for (int i = 0; i < s->n_frames; i++) fprintf(f, i ? ",%d" : "%d", s->frames[i].queued);

    fprintf(f, "]},\n\"frames\":[");
    for (int i = 0; i < s->n_frames; i++) {
        const Frame *fr = &s->frames[i];
        fprintf(f, "%s{\"t\":%.1f,\"v\":", i ? ",\n" : "", fr->t);
        write_ints(f, fr->veh, fr->n_veh * 4);
        fprintf(f, ",\"l\":\"%s\",\"c\":[", fr->lights);
        for (int e = 0; e < g->n_edges; e++) fprintf(f, e ? ",%d" : "%d", fr->cong[e]);
        fprintf(f, "]}");
    }
    fprintf(f, "]}");
}

int export_js(const char *path, const Graph *g, Simulation *sims[2], const SimStats stats[2],
              const AlgoSummary dist[ALGO_COUNT], const AlgoSummary time[ALGO_COUNT], const BenchRow bench[BENCH_SIZES])
{
    FILE *f = fopen(path, "w");
    if (!f) return -1;

    fprintf(f, "window.SIM_DATA = {\n");
    write_graph(f, g);
    fprintf(f, ",\n\"signals\":[");
    for (int i = 0; i < sims[0]->n_signals; i++) fprintf(f, i ? ",%d" : "%d", sims[0]->signals[i].node);
    fprintf(f, "],\n\"algorithms\":{\"distance\":");
    write_summary(f, dist);
    fprintf(f, ",\"time\":");
    write_summary(f, time);
    fprintf(f, "},\n");
    write_bench(f, bench);
    fprintf(f, ",\n");
    write_planner(f, g);
    fprintf(f, ",\n\"sims\":[");
    write_sim(f, sims[0], &stats[0]);
    fprintf(f, ",\n");
    write_sim(f, sims[1], &stats[1]);
    fprintf(f, "]\n};\n");

    fclose(f);
    return 0;
}
