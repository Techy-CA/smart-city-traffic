#include "analytics.h"
#include "util.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void compare_algorithms(const Graph *g, WeightMode mode, AlgoSummary out[ALGO_COUNT])
{
    memset(out, 0, sizeof(AlgoSummary) * ALGO_COUNT);
    for (int a = 0; a < ALGO_COUNT; a++) out[a].algo = a;

    for (int s = 0; s < g->n_nodes; s++)
        for (int d = 0; d < g->n_nodes; d++) {
            if (s == d) continue;
            PathResult ref = find_path(g, s, d, mode, ALGO_DIJKSTRA, 0);
            if (!ref.found) {
                path_result_free(&ref);
                continue;
            }
            for (int a = 0; a < ALGO_COUNT; a++) {
                PathResult r  = find_path(g, s, d, mode, a, 0);
                AlgoSummary *o = &out[a];
                o->queries++;
                o->avg_visited    += r.visited;
                o->avg_relax      += r.relaxations;
                o->avg_pq_ops     += r.pq_ops;
                o->avg_runtime_us += r.runtime_us;
                o->avg_distance_m += r.distance_m;
                if (fabs(r.cost - ref.cost) < 1e-6 * (1 + ref.cost)) o->optimal_pct += 1;
                o->avg_excess_pct += 100.0 * (r.cost - ref.cost) / ref.cost;
                path_result_free(&r);
            }
            path_result_free(&ref);
        }

    for (int a = 0; a < ALGO_COUNT; a++) {
        AlgoSummary *o = &out[a];
        if (!o->queries) continue;
        o->avg_visited    /= o->queries;
        o->avg_relax      /= o->queries;
        o->avg_pq_ops     /= o->queries;
        o->avg_runtime_us /= o->queries;
        o->avg_distance_m /= o->queries;
        o->optimal_pct     = 100.0 * o->optimal_pct / o->queries;
        o->avg_excess_pct /= o->queries;
    }
}

void print_algo_summary(const AlgoSummary s[ALGO_COUNT], WeightMode mode)
{
    printf("\n  Algorithm comparison over all %d source/destination pairs (%s weights)\n\n", s[0].queries,
           mode == WEIGHT_DISTANCE ? "distance" : "travel-time");
    printf("  %-18s %9s %10s %9s %10s %9s %10s\n", "Algorithm", "Visited", "Relaxed", "PQ ops", "Time(us)",
           "Optimal", "Extra cost");
    printf("  ------------------ --------- ---------- --------- ---------- --------- ----------\n");
    for (int a = 0; a < ALGO_COUNT; a++)
        printf("  %-18s %9.1f %10.1f %9.1f %10.2f %8.0f%% %9.2f%%\n", algo_name(a), s[a].avg_visited,
               s[a].avg_relax, s[a].avg_pq_ops, s[a].avg_runtime_us, s[a].optimal_pct, s[a].avg_excess_pct);
}

void compare_route(const Graph *g, int src, int dst, WeightMode mode)
{
    printf("\n  %s -> %s (%s)\n\n", g->nodes[src].name, g->nodes[dst].name,
           mode == WEIGHT_DISTANCE ? "shortest distance" : "fastest at current traffic");
    printf("  %-18s %9s %8s %6s %8s %9s %10s\n", "Algorithm", "Dist(km)", "ETA(min)", "Hops", "Visited",
           "Relaxed", "Time(us)");
    printf("  ------------------ --------- -------- ------ -------- --------- ----------\n");
    for (int a = 0; a < ALGO_COUNT; a++) {
        PathResult r = find_path(g, src, dst, mode, a, 0);
        if (r.found)
            printf("  %-18s %9.2f %8.1f %6d %8d %9ld %10.2f\n", algo_name(a), r.distance_m / 1000, r.time_s / 60,
                   r.path_len - 1, r.visited, r.relaxations, r.runtime_us);
        else
            printf("  %-18s   no route\n", algo_name(a));
        path_result_free(&r);
    }
}

/* Random far-apart queries on generated grid cities of growing size. */
int benchmark_scaling(BenchRow rows[BENCH_SIZES], int verbose)
{
    static const int sizes[BENCH_SIZES] = {10, 20, 40, 70, 100};
    const int queries = 6;

    for (int i = 0; i < BENCH_SIZES; i++) {
        int    n = sizes[i];
        Graph *g = graph_create_grid(n, n, 1000 + n);
        BenchRow *b = &rows[i];
        memset(b, 0, sizeof *b);
        b->rows = b->cols = n;
        b->V    = g->n_nodes;
        b->E    = g->n_edges;
        if (verbose) printf("  grid %3dx%-3d  V=%-6d E=%-6d ...", n, n, b->V, b->E), fflush(stdout);

        /* small graphs finish in microseconds: repeat for a stable timing */
        int reps = b->V < 1000 ? 20 : b->V < 3000 ? 4 : 1;
        rng_seed(77 + n);
        for (int q = 0; q < queries; q++) {
            int src = rng_int(0, n / 4) * n + rng_int(0, n / 4);                 /* top-left area   */
            int dst = rng_int(3 * n / 4, n - 1) * n + rng_int(3 * n / 4, n - 1); /* bottom-right    */
            for (int a = 0; a < ALGO_COUNT; a++)
                for (int r = 0; r < reps; r++) {
                    PathResult res = find_path(g, src, dst, WEIGHT_TIME, a, 0);
                    b->runtime_us[a] += res.runtime_us / (queries * reps);
                    b->visited[a]    += (double)res.visited / (queries * reps);
                    path_result_free(&res);
                }
        }
        if (verbose) printf(" done\n");
        graph_free(g);
    }
    return BENCH_SIZES;
}

void print_bench(const BenchRow *rows, int n)
{
    printf("\n  Scaling benchmark: average runtime per query (microseconds)\n\n");
    printf("  %-8s %7s", "V", "E");
    for (int a = 0; a < ALGO_COUNT; a++) printf(" %17s", algo_name(a));
    printf("\n  -------- -------");
    for (int a = 0; a < ALGO_COUNT; a++) printf(" -----------------");
    printf("\n");
    for (int i = 0; i < n; i++) {
        printf("  %-8d %7d", rows[i].V, rows[i].E);
        for (int a = 0; a < ALGO_COUNT; a++) printf(" %17.1f", rows[i].runtime_us[a]);
        printf("\n");
    }
    printf("\n  Nodes explored per query\n\n  %-8s", "V");
    for (int a = 0; a < ALGO_COUNT; a++) printf(" %17s", algo_name(a));
    printf("\n");
    for (int i = 0; i < n; i++) {
        printf("  %-8d", rows[i].V);
        for (int a = 0; a < ALGO_COUNT; a++) printf(" %17.0f", rows[i].visited[a]);
        printf("\n");
    }
}
