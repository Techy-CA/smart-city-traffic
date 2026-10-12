#ifndef EXPORT_H
#define EXPORT_H

#include "analytics.h"
#include "graph.h"
#include "traffic.h"

/* Everything the browser dashboard needs, produced by the C core. */
typedef struct {
    const Graph       *g;
    Simulation        *sims[2];   /* [0] without corridor, [1] with corridor */
    SimStats           stats[2];
    AlgoSummary        dist_summary[ALGO_COUNT];
    AlgoSummary        time_summary[ALGO_COUNT];
    const BenchRow    *bench;
    int                n_bench;
} ExportData;

/* Writes `window.SIM_DATA = {...};` so web/index.html works straight from
 * the file system (no web server needed). Returns 0 on success. */
int export_js(const char *path, const ExportData *d);

#endif
