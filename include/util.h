#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>

#define INF 1e18

/* Deterministic RNG (xorshift32) so every run of the simulation is reproducible
 * on every compiler/OS -- rand() differs between libc implementations. */
void     rng_seed(unsigned int seed);
unsigned rng_next(void);
int      rng_int(int lo, int hi);          /* inclusive range            */
double   rng_double(void);                 /* uniform in [0, 1)          */

double   now_us(void);                     /* monotonic clock, microseconds */
double   euclid(double x1, double y1, double x2, double y2);

void    *xmalloc(size_t n);
void    *xcalloc(size_t count, size_t size);
void    *xrealloc(void *p, size_t n);

#endif
