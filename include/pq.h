#ifndef PQ_H
#define PQ_H

/*
 * Indexed binary min-heap (priority queue with decrease-key).
 *
 *   heap[]  : node ids arranged as a binary heap on key[]
 *   pos[v]  : index of v inside heap[], or -1 if v is not in the heap
 *   key[v]  : current priority of v
 *
 *   push / decrease_key / pop  : O(log V)
 *   contains / peek            : O(1)
 */
typedef struct {
    int    *heap;
    int    *pos;
    double *key;
    int     size, capacity;
    long    ops;   /* number of push + decrease-key operations (for analytics) */
} MinHeap;

MinHeap *pq_create(int capacity);
void     pq_free(MinHeap *pq);
int      pq_empty(const MinHeap *pq);
int      pq_contains(const MinHeap *pq, int v);
void     pq_push(MinHeap *pq, int v, double key);
void     pq_decrease_key(MinHeap *pq, int v, double key);
void     pq_push_or_decrease(MinHeap *pq, int v, double key);
int      pq_pop(MinHeap *pq);  /* removes and returns the id with the smallest key */

/*
 * Circular FIFO queue of ints. Used by BFS and by every traffic-light stop
 * line (vehicles discharge first-come-first-served).
 */
typedef struct {
    int *data;
    int  head, size, capacity;
} Queue;

Queue *queue_create(int capacity);
void   queue_free(Queue *q);
void   queue_push(Queue *q, int v);
int    queue_pop(Queue *q);
int    queue_peek(const Queue *q);
int    queue_at(const Queue *q, int i); /* i-th element from the front */
void   queue_remove(Queue *q, int v);   /* remove arbitrary element, keeps order */
int    queue_index_of(const Queue *q, int v);

#endif
