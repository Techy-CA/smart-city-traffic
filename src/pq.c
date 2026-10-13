#include "city.h"

#include <stdlib.h>

MinHeap *heap_create(int n)
{
    MinHeap *h = malloc(sizeof(MinHeap));
    h->heap = malloc(sizeof(int) * n);
    h->pos = malloc(sizeof(int) * n);
    h->key = malloc(sizeof(double) * n);
    h->size = 0;
    h->ops = 0;
    for (int i = 0; i < n; i++) h->pos[i] = -1;
    return h;
}

void heap_free(MinHeap *h)
{
    free(h->heap);
    free(h->pos);
    free(h->key);
    free(h);
}

static void swap(MinHeap *h, int i, int j)
{
    int a = h->heap[i], b = h->heap[j];
    h->heap[i] = b;
    h->heap[j] = a;
    h->pos[b] = i;
    h->pos[a] = j;
}

static void sift_up(MinHeap *h, int i)
{
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (h->key[h->heap[parent]] <= h->key[h->heap[i]]) break;
        swap(h, i, parent);
        i = parent;
    }
}

static void sift_down(MinHeap *h, int i)
{
    while (1) {
        int left = 2 * i + 1, right = 2 * i + 2, small = i;
        if (left < h->size && h->key[h->heap[left]] < h->key[h->heap[small]]) small = left;
        if (right < h->size && h->key[h->heap[right]] < h->key[h->heap[small]]) small = right;
        if (small == i) return;
        swap(h, i, small);
        i = small;
    }
}

void heap_update(MinHeap *h, int v, double key)
{
    if (h->pos[v] < 0) {
        h->heap[h->size] = v;
        h->pos[v] = h->size;
        h->size++;
    }
    h->key[v] = key;
    sift_up(h, h->pos[v]);
    h->ops++;
}

int heap_pop(MinHeap *h)
{
    int top = h->heap[0];
    h->pos[top] = -1;
    h->size--;
    if (h->size > 0) {
        h->heap[0] = h->heap[h->size];
        h->pos[h->heap[0]] = 0;
        sift_down(h, 0);
    }
    return top;
}

Queue *queue_create(int capacity)
{
    Queue *q = malloc(sizeof(Queue));
    q->data = malloc(sizeof(int) * capacity);
    q->head = 0;
    q->size = 0;
    q->capacity = capacity;
    return q;
}

void queue_free(Queue *q)
{
    free(q->data);
    free(q);
}

void queue_push(Queue *q, int v)
{
    if (q->size == q->capacity) {
        int *bigger = malloc(sizeof(int) * q->capacity * 2);
        for (int i = 0; i < q->size; i++) bigger[i] = q->data[(q->head + i) % q->capacity];
        free(q->data);
        q->data = bigger;
        q->head = 0;
        q->capacity *= 2;
    }
    q->data[(q->head + q->size) % q->capacity] = v;
    q->size++;
}

int queue_pop(Queue *q)
{
    int v = q->data[q->head];
    q->head = (q->head + 1) % q->capacity;
    q->size--;
    return v;
}

int queue_find(const Queue *q, int v)
{
    for (int i = 0; i < q->size; i++)
        if (q->data[(q->head + i) % q->capacity] == v) return i;
    return -1;
}
