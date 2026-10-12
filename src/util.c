#define _POSIX_C_SOURCE 199309L
#include "util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static unsigned int rng_state = 2463534242u;

void rng_seed(unsigned int seed) { rng_state = seed ? seed : 2463534242u; }

unsigned rng_next(void)
{
    unsigned x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return rng_state = x;
}

int rng_int(int lo, int hi) { return lo + (int)(rng_next() % (unsigned)(hi - lo + 1)); }

double rng_double(void) { return (rng_next() & 0xFFFFFF) / (double)0x1000000; }

double now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

double euclid(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2, dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

void *xmalloc(size_t n)
{
    void *p = malloc(n ? n : 1);
    if (!p) { fprintf(stderr, "out of memory\n"); exit(1); }
    return p;
}

void *xcalloc(size_t count, size_t size)
{
    void *p = calloc(count ? count : 1, size ? size : 1);
    if (!p) { fprintf(stderr, "out of memory\n"); exit(1); }
    return p;
}

void *xrealloc(void *p, size_t n)
{
    p = realloc(p, n ? n : 1);
    if (!p) { fprintf(stderr, "out of memory\n"); exit(1); }
    return p;
}
