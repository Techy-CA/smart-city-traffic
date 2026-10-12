#include "pq.h"
#include "util.h"

#include <stdlib.h>

/* ------------------------------------------------------------ min-heap */

MinHeap *pq_create(int capacity)
{
    MinHeap *pq  = xmalloc(sizeof(MinHeap));
    pq->heap     = xmalloc(sizeof(int) * capacity);
    pq->pos      = xmalloc(sizeof(int) * capacity);
    pq->key      = xmalloc(sizeof(double) * capacity);
    pq->size     = 0;
    pq->capacity = capacity;
    pq->ops      = 0;
    for (int i = 0; i < capacity; i++) pq->pos[i] = -1;
    return pq;
}

void pq_free(MinHeap *pq)
{
    if (!pq) return;
    free(pq->heap);
    free(pq->pos);
    free(pq->key);
    free(pq);
}

int pq_empty(const MinHeap *pq) { return pq->size == 0; }
int pq_contains(const MinHeap *pq, int v) { return pq->pos[v] >= 0; }

static void swap_nodes(MinHeap *pq, int i, int j)
{
    int a = pq->heap[i], b = pq->heap[j];
    pq->heap[i] = b;
    pq->heap[j] = a;
    pq->pos[b]  = i;
    pq->pos[a]  = j;
}

static void sift_up(MinHeap *pq, int i)
{
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->key[pq->heap[parent]] <= pq->key[pq->heap[i]]) break;
        swap_nodes(pq, i, parent);
        i = parent;
    }
}

static void sift_down(MinHeap *pq, int i)
{
    for (;;) {
        int l = 2 * i + 1, r = l + 1, m = i;
        if (l < pq->size && pq->key[pq->heap[l]] < pq->key[pq->heap[m]]) m = l;
        if (r < pq->size && pq->key[pq->heap[r]] < pq->key[pq->heap[m]]) m = r;
        if (m == i) return;
        swap_nodes(pq, i, m);
        i = m;
    }
}

void pq_push(MinHeap *pq, int v, double key)
{
    pq->key[v]          = key;
    pq->heap[pq->size]  = v;
    pq->pos[v]          = pq->size;
    sift_up(pq, pq->size++);
    pq->ops++;
}

void pq_decrease_key(MinHeap *pq, int v, double key)
{
    pq->key[v] = key;
    sift_up(pq, pq->pos[v]);
    pq->ops++;
}

void pq_push_or_decrease(MinHeap *pq, int v, double key)
{
    if (pq_contains(pq, v)) pq_decrease_key(pq, v, key);
    else                    pq_push(pq, v, key);
}

int pq_pop(MinHeap *pq)
{
    int top     = pq->heap[0];
    pq->pos[top] = -1;
    if (--pq->size > 0) {
        pq->heap[0]            = pq->heap[pq->size];
        pq->pos[pq->heap[0]]   = 0;
        sift_down(pq, 0);
    }
    return top;
}

/* ------------------------------------------------------------ queue */

Queue *queue_create(int capacity)
{
    Queue *q    = xmalloc(sizeof(Queue));
    q->data     = xmalloc(sizeof(int) * capacity);
    q->head     = 0;
    q->size     = 0;
    q->capacity = capacity;
    return q;
}

void queue_free(Queue *q)
{
    if (!q) return;
    free(q->data);
    free(q);
}

void queue_push(Queue *q, int v)
{
    if (q->size == q->capacity) { /* grow: unroll the ring into a bigger buffer */
        int *d = xmalloc(sizeof(int) * q->capacity * 2);
        for (int i = 0; i < q->size; i++) d[i] = q->data[(q->head + i) % q->capacity];
        free(q->data);
        q->data     = d;
        q->head     = 0;
        q->capacity *= 2;
    }
    q->data[(q->head + q->size++) % q->capacity] = v;
}

int queue_pop(Queue *q)
{
    int v   = q->data[q->head];
    q->head = (q->head + 1) % q->capacity;
    q->size--;
    return v;
}

int queue_peek(const Queue *q) { return q->data[q->head]; }
int queue_at(const Queue *q, int i) { return q->data[(q->head + i) % q->capacity]; }

int queue_index_of(const Queue *q, int v)
{
    for (int i = 0; i < q->size; i++)
        if (queue_at(q, i) == v) return i;
    return -1;
}

void queue_remove(Queue *q, int v)
{
    int i = queue_index_of(q, v);
    if (i < 0) return;
    for (; i < q->size - 1; i++)
        q->data[(q->head + i) % q->capacity] = q->data[(q->head + i + 1) % q->capacity];
    q->size--;
}
