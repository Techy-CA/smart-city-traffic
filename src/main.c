#include "city.h"

#include <stdlib.h>
#include <string.h>

#define MAP_FILE "data/city_map.txt"
#define EXPORT_FILE "web/data/sim_data.js"

static int read_int(const char *prompt, int lo, int hi)
{
    char line[64];
    while (1) {
        printf("%s", prompt);
        if (!fgets(line, sizeof line, stdin)) exit(0);
        int v;
        if (sscanf(line, "%d", &v) == 1 && v >= lo && v <= hi) return v;
        printf("  Enter a number between %d and %d.\n", lo, hi);
    }
}

static void list_nodes(const Graph *g)
{
    printf("\n");
    for (int i = 0; i < g->n_nodes; i++) printf("  %2d %-18s%s", i, g->nodes[i].name, i % 3 == 2 ? "\n" : "");
    printf("\n");
}

static void run_simulations(Graph *g, Simulation *sims[2], SimStats st[2])
{
    for (int c = 0; c < 2; c++) {
        sims[c] = sim_create(g, c);
        sim_run(sims[c]);
        st[c] = sim_stats(sims[c]);
        sim_print_stats(sims[c], &st[c]);
    }
    reset_load(g);
    double saved = st[0].amb_time_s - st[1].amb_time_s;
    printf("\n  >> Green corridor saved %.0f s (%.0f%% faster)\n", saved, 100.0 * saved / st[0].amb_time_s);
}

static void export_all(Graph *g)
{
    Simulation *sims[2];
    SimStats st[2];
    AlgoSummary dist[ALGO_COUNT], time[ALGO_COUNT];
    BenchRow bench[BENCH_SIZES];

    printf("\n  [1/4] Traffic simulation\n");
    run_simulations(g, sims, st);
    printf("\n  [2/4] Comparing algorithms on every route\n");
    compare_algorithms(g, BY_DISTANCE, dist);
    print_algo_summary(dist, BY_DISTANCE);
    compare_algorithms(g, BY_TIME, time);
    printf("\n  [3/4] Scaling benchmark\n");
    benchmark_scaling(bench);
    print_bench(bench);
    printf("\n  [4/4] Writing %s\n", EXPORT_FILE);

    if (export_js(EXPORT_FILE, g, sims, st, dist, time, bench) == 0)
        printf("  Done. Open web/index.html in a browser.\n");
    else
        printf("  Could not write %s\n", EXPORT_FILE);
    sim_free(sims[0]);
    sim_free(sims[1]);
}

static void find_route(Graph *g, int compare_all)
{
    list_nodes(g);
    int src = read_int("  From (id): ", 0, g->n_nodes - 1);
    int dst = read_int("  To   (id): ", 0, g->n_nodes - 1);
    int mode = read_int("  0 = shortest distance, 1 = fastest time: ", 0, 1);
    if (compare_all) {
        compare_route(g, src, dst, mode);
        return;
    }
    for (int a = 0; a < ALGO_COUNT; a++) printf("  %d. %s\n", a, ALGO_NAME[a]);
    int algo = read_int("  Algorithm: ", 0, ALGO_COUNT - 1);
    PathResult r = find_path(g, src, dst, mode, algo);
    printf("\n");
    path_print(g, &r, algo);
    path_free(&r);
}

static void toggle_road(Graph *g)
{
    list_nodes(g);
    int u = read_int("  Road from (id): ", 0, g->n_nodes - 1);
    int v = read_int("  Road to   (id): ", 0, g->n_nodes - 1);
    int e = find_edge(g, u, v);
    if (e < 0) {
        printf("  There is no direct road between them.\n");
        return;
    }
    int closed = !g->edges[e].closed;
    g->edges[e].closed = closed;
    g->edges[g->edges[e].reverse].closed = closed;
    printf("  %s <-> %s is now %s.\n", g->nodes[u].name, g->nodes[v].name, closed ? "CLOSED" : "open");
}

static void menu(Graph *g)
{
    while (1) {
        printf("\n  1. Show city map\n"
               "  2. Find a route\n"
               "  3. Compare all algorithms on one route\n"
               "  4. Close / reopen a road\n"
               "  5. Run traffic + ambulance simulation\n"
               "  6. Algorithm analytics over all routes\n"
               "  7. Scaling benchmark\n"
               "  8. Export everything for the web dashboard\n"
               "  0. Exit\n");
        int choice = read_int("\n  Choice: ", 0, 8);

        if (choice == 0) return;
        if (choice == 1) graph_print(g);
        if (choice == 2) find_route(g, 0);
        if (choice == 3) find_route(g, 1);
        if (choice == 4) toggle_road(g);
        if (choice == 5) {
            Simulation *sims[2];
            SimStats st[2];
            run_simulations(g, sims, st);
            sim_free(sims[0]);
            sim_free(sims[1]);
        }
        if (choice == 6) {
            AlgoSummary s[ALGO_COUNT];
            compare_algorithms(g, BY_DISTANCE, s);
            print_algo_summary(s, BY_DISTANCE);
            compare_algorithms(g, BY_TIME, s);
            print_algo_summary(s, BY_TIME);
        }
        if (choice == 7) {
            BenchRow rows[BENCH_SIZES];
            benchmark_scaling(rows);
            print_bench(rows);
        }
        if (choice == 8) export_all(g);
    }
}

int main(int argc, char **argv)
{
    Graph *g = graph_load(MAP_FILE);
    if (!g) {
        printf("Cannot open %s\n", MAP_FILE);
        return 1;
    }
    printf("\n  SMART CITY TRAFFIC INTELLIGENCE SYSTEM\n");
    printf("  Loaded %d intersections and %d roads\n", g->n_nodes, g->n_edges / 2);

    if (argc > 1 && strcmp(argv[1], "--export") == 0) export_all(g);
    else menu(g);

    graph_free(g);
    return 0;
}
