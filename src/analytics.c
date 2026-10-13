#include "city.h"

#include <math.h>
#include <string.h>

void compare_algorithms(const Graph *g, WeightMode mode, AlgoSummary out[ALGO_COUNT])
{
    memset(out, 0, sizeof(AlgoSummary) * ALGO_COUNT);

    for (int s = 0; s < g->n_nodes; s++)
        for (int d = 0; d < g->n_nodes; d++) {
            if (s == d) continue;
            PathResult best = find_path(g, s, d, mode, DIJKSTRA);
            if (best.found) {
                for (int a = 0; a < ALGO_COUNT; a++) {
                    PathResult r = find_path(g, s, d, mode, a);
                    AlgoSummary *o = &out[a];
                    o->queries++;
                    o->visited += r.visited;
                    o->relax += r.relaxations;
                    o->pq_ops += r.pq_ops;
                    o->runtime_us += r.runtime_us;
                    o->distance_m += r.distance_m;
                    if (fabs(r.cost - best.cost) < 1e-6 * (1 + best.cost)) o->optimal_pct += 1;
                    o->excess_pct += 100.0 * (r.cost - best.cost) / best.cost;
                    path_free(&r);
                }
            }
            path_free(&best);
        }

    for (int a = 0; a < ALGO_COUNT; a++) {
        AlgoSummary *o = &out[a];
        if (o->queries == 0) continue;
        o->visited /= o->queries;
        o->relax /= o->queries;
        o->pq_ops /= o->queries;
        o->runtime_us /= o->queries;
        o->distance_m /= o->queries;
        o->optimal_pct = 100.0 * o->optimal_pct / o->queries;
        o->excess_pct /= o->queries;
    }
}

void print_algo_summary(const AlgoSummary s[ALGO_COUNT], WeightMode mode)
{
    printf("\n  All %d routes, %s weights\n\n", s[0].queries, mode == BY_DISTANCE ? "distance" : "travel-time");
    printf("  %-18s %9s %10s %9s %10s %9s %10s\n", "Algorithm", "Visited", "Relaxed", "PQ ops", "Time(us)", "Optimal",
           "Extra cost");
    for (int a = 0; a < ALGO_COUNT; a++)
        printf("  %-18s %9.1f %10.1f %9.1f %10.2f %8.0f%% %9.2f%%\n", ALGO_NAME[a], s[a].visited, s[a].relax,
               s[a].pq_ops, s[a].runtime_us, s[a].optimal_pct, s[a].excess_pct);
}

void compare_route(const Graph *g, int src, int dst, WeightMode mode)
{
    printf("\n  %s -> %s\n\n", g->nodes[src].name, g->nodes[dst].name);
    printf("  %-18s %9s %8s %6s %8s %9s %10s\n", "Algorithm", "Dist(km)", "ETA(min)", "Hops", "Visited", "Relaxed",
           "Time(us)");
    for (int a = 0; a < ALGO_COUNT; a++) {
        PathResult r = find_path(g, src, dst, mode, a);
        if (r.found)
            printf("  %-18s %9.2f %8.1f %6d %8d %9ld %10.2f\n", ALGO_NAME[a], r.distance_m / 1000, r.time_s / 60,
                   r.path_len - 1, r.visited, r.relaxations, r.runtime_us);
        else
            printf("  %-18s   no route\n", ALGO_NAME[a]);
        path_free(&r);
    }
}

void benchmark_scaling(BenchRow rows[BENCH_SIZES])
{
    static const int sizes[BENCH_SIZES] = {10, 20, 40, 70, 100};
    const int queries = 6;

    for (int i = 0; i < BENCH_SIZES; i++) {
        int n = sizes[i];
        Graph *g = graph_create_grid(n, 1000 + n);
        BenchRow *b = &rows[i];
        memset(b, 0, sizeof *b);
        b->V = g->n_nodes;
        b->E = g->n_edges;
        printf("  grid %3dx%-3d  V=%-6d E=%-6d\n", n, n, b->V, b->E);

        int reps = 1;
        if (b->V < 3000) reps = 4;
        if (b->V < 1000) reps = 20;

        rng_seed(77 + n);
        for (int q = 0; q < queries; q++) {
            int src_row = rng_int(0, n / 4);
            int src_col = rng_int(0, n / 4);
            int dst_row = rng_int(3 * n / 4, n - 1);
            int dst_col = rng_int(3 * n / 4, n - 1);
            for (int a = 0; a < ALGO_COUNT; a++)
                for (int k = 0; k < reps; k++) {
                    PathResult r = find_path(g, src_row * n + src_col, dst_row * n + dst_col, BY_TIME, a);
                    b->runtime_us[a] += r.runtime_us / (queries * reps);
                    b->visited[a] += (double)r.visited / (queries * reps);
                    path_free(&r);
                }
        }
        graph_free(g);
    }
}

void print_bench(const BenchRow rows[BENCH_SIZES])
{
    printf("\n  Average runtime per query (microseconds)\n\n  %-8s", "V");
    for (int a = 0; a < ALGO_COUNT; a++) printf(" %17s", ALGO_NAME[a]);
    printf("\n");
    for (int i = 0; i < BENCH_SIZES; i++) {
        printf("  %-8d", rows[i].V);
        for (int a = 0; a < ALGO_COUNT; a++) printf(" %17.1f", rows[i].runtime_us[a]);
        printf("\n");
    }
}
