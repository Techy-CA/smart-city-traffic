#ifndef ANALYTICS_H
#define ANALYTICS_H

#include "algorithms.h"
#include "graph.h"

/* Averages over every (src, dst) pair of the city map. */
typedef struct {
    Algorithm algo;
    int       queries;
    double    avg_visited, avg_relax, avg_pq_ops, avg_runtime_us;
    double    avg_distance_m;
    double    optimal_pct;       /* % of queries whose path matches the true optimum */
    double    avg_excess_pct;    /* extra distance vs the optimum, averaged          */
} AlgoSummary;

#define BENCH_SIZES 5

/* One row of the scaling experiment on generated grid cities. */
typedef struct {
    int    rows, cols, V, E;
    double runtime_us[ALGO_COUNT];
    double visited[ALGO_COUNT];
} BenchRow;

void compare_algorithms(const Graph *g, WeightMode mode, AlgoSummary out[ALGO_COUNT]);
void print_algo_summary(const AlgoSummary s[ALGO_COUNT], WeightMode mode);

int  benchmark_scaling(BenchRow rows[BENCH_SIZES], int verbose);
void print_bench(const BenchRow *rows, int n);

void compare_route(const Graph *g, int src, int dst, WeightMode mode); /* one query, all algorithms */

#endif
