/*
 * Smart City Traffic Intelligence System
 *
 *   ./traffic                interactive menu
 *   ./traffic --export       run everything, write web/data/sim_data.js
 *   ./traffic --help
 */
#include "algorithms.h"
#include "analytics.h"
#include "export.h"
#include "graph.h"
#include "traffic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_MAP    "data/city_map.txt"
#define DEFAULT_EXPORT "web/data/sim_data.js"

static void banner(void)
{
    printf("\n  +--------------------------------------------------------+\n");
    printf("  |      SMART CITY TRAFFIC INTELLIGENCE SYSTEM  (C/DSA)    |\n");
    printf("  |  graphs - priority queues - shortest paths - simulation |\n");
    printf("  +--------------------------------------------------------+\n");
}

static int read_int(const char *prompt, int lo, int hi)
{
    char line[64];
    for (;;) {
        printf("%s", prompt);
        fflush(stdout);
        if (!fgets(line, sizeof line, stdin)) exit(0);
        char *end;
        long v = strtol(line, &end, 10);
        if (end != line && v >= lo && v <= hi) return (int)v;
        printf("  Enter a number between %d and %d.\n", lo, hi);
    }
}

static void list_nodes(const Graph *g)
{
    printf("\n");
    for (int i = 0; i < g->n_nodes; i++)
        printf("  %2d %-18s%s", i, g->nodes[i].name, (i % 3 == 2) ? "\n" : "");
    printf("\n");
}

static void run_simulations(Graph *g, Simulation *sims[2], SimStats st[2], int verbose)
{
    for (int c = 0; c < 2; c++) {
        if (verbose) printf("\n  Simulating %s green corridor ...\n", c ? "WITH" : "WITHOUT");
        sims[c] = sim_create(g, sim_default_config(c));
        sim_run(sims[c]);
        st[c] = sim_stats(sims[c]);
        if (verbose) sim_print_stats(g, sims[c], &st[c]);
    }
    graph_reset_load(g); /* back to an empty city for route queries */
    if (verbose && st[0].amb_arrived && st[1].amb_arrived)
        printf("\n  >> Green corridor saved %.0f s (%.0f%% faster response)\n", st[0].amb_time_s - st[1].amb_time_s,
               100.0 * (st[0].amb_time_s - st[1].amb_time_s) / st[0].amb_time_s);
}

static int export_all(Graph *g, const char *out)
{
    ExportData d;
    memset(&d, 0, sizeof d);
    d.g = g;

    printf("\n  [1/4] Traffic simulation (2 runs) ...\n");
    run_simulations(g, d.sims, d.stats, 1);

    printf("\n  [2/4] Comparing algorithms on every route of the city ...\n");
    compare_algorithms(g, WEIGHT_DISTANCE, d.dist_summary);
    print_algo_summary(d.dist_summary, WEIGHT_DISTANCE);
    compare_algorithms(g, WEIGHT_TIME, d.time_summary);

    printf("\n  [3/4] Scaling benchmark on generated grid cities ...\n");
    BenchRow bench[BENCH_SIZES];
    d.n_bench = benchmark_scaling(bench, 1);
    d.bench   = bench;
    print_bench(bench, d.n_bench);

    printf("\n  [4/4] Writing %s ...\n", out);
    int rc = export_js(out, &d);
    sim_free(d.sims[0]);
    sim_free(d.sims[1]);
    if (rc) {
        fprintf(stderr, "  could not write %s (does the directory exist?)\n", out);
        return 1;
    }
    printf("  Done. Open web/index.html in a browser.\n\n");
    return 0;
}

static void menu(Graph *g, const char *export_path)
{
    for (;;) {
        printf("\n  1. Show city map (adjacency list)\n"
               "  2. Find a route (pick algorithm)\n"
               "  3. Compare all algorithms on one route\n"
               "  4. Close / reopen a road (accident, roadwork)\n"
               "  5. Run traffic + ambulance simulation\n"
               "  6. Algorithm analytics over all routes\n"
               "  7. Scaling benchmark (100 -> 10,000 intersections)\n"
               "  8. Export everything for the web dashboard\n"
               "  0. Exit\n");
        int choice = read_int("\n  Choice: ", 0, 8);
        switch (choice) {
        case 0: return;
        case 1: graph_print(g); break;
        case 2:
        case 3: {
            list_nodes(g);
            int src  = read_int("  From (id): ", 0, g->n_nodes - 1);
            int dst  = read_int("  To   (id): ", 0, g->n_nodes - 1);
            int mode = read_int("  Optimise for 0 = distance, 1 = travel time: ", 0, 1);
            if (choice == 3) {
                compare_route(g, src, dst, mode);
                break;
            }
            for (int a = 0; a < ALGO_COUNT; a++) printf("  %d. %s\n", a, algo_name(a));
            int algo     = read_int("  Algorithm: ", 0, ALGO_COUNT - 1);
            PathResult r = find_path(g, src, dst, mode, algo, 1);
            printf("\n");
            path_print(g, &r);
            if (r.visit_count) {
                printf("  Exploration order: ");
                for (int i = 0; i < r.visit_count; i++) printf("%d%s", r.visit_order[i], i + 1 < r.visit_count ? " " : "\n");
            }
            path_result_free(&r);
            break;
        }
        case 4: {
            list_nodes(g);
            int u = read_int("  Road from (id): ", 0, g->n_nodes - 1);
            int v = read_int("  Road to   (id): ", 0, g->n_nodes - 1);
            int e = graph_find_edge(g, u, v);
            if (e < 0) {
                printf("  There is no direct road between those intersections.\n");
                break;
            }
            int closed = !g->edges[e].closed;
            g->edges[e].closed = closed;
            if (g->edges[e].reverse >= 0) g->edges[g->edges[e].reverse].closed = closed;
            printf("  %s <-> %s is now %s. Routes will avoid it.\n", g->nodes[u].name, g->nodes[v].name,
                   closed ? "CLOSED" : "open");
            break;
        }
        case 5: {
            Simulation *sims[2];
            SimStats    st[2];
            run_simulations(g, sims, st, 1);
            sim_free(sims[0]);
            sim_free(sims[1]);
            break;
        }
        case 6: {
            AlgoSummary s[ALGO_COUNT];
            compare_algorithms(g, WEIGHT_DISTANCE, s);
            print_algo_summary(s, WEIGHT_DISTANCE);
            compare_algorithms(g, WEIGHT_TIME, s);
            print_algo_summary(s, WEIGHT_TIME);
            break;
        }
        case 7: {
            BenchRow rows[BENCH_SIZES];
            print_bench(rows, benchmark_scaling(rows, 1));
            break;
        }
        case 8: export_all(g, export_path); break;
        }
    }
}

int main(int argc, char **argv)
{
    const char *map = DEFAULT_MAP, *out = DEFAULT_EXPORT;
    int do_export = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--export")) do_export = 1;
        else if (!strcmp(argv[i], "--map") && i + 1 < argc) map = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else {
            printf("usage: %s [--export] [--map FILE] [--out FILE]\n", argv[0]);
            return strcmp(argv[i], "--help") ? 1 : 0;
        }
    }

    Graph *g = graph_load(map);
    if (!g) {
        fprintf(stderr, "cannot load map '%s'\n", map);
        return 1;
    }
    banner();
    printf("  Loaded %s: %d intersections, %d road segments\n", map, g->n_nodes, g->n_edges);

    int rc = 0;
    if (do_export) rc = export_all(g, out);
    else           menu(g, out);
    graph_free(g);
    return rc;
}
