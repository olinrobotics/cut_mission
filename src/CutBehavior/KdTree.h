#ifndef KDTREE_H
#define KDTREE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_DIM 2

struct kd_node_t{
    double x[MAX_DIM];
    double value;
    struct kd_node_t *left, *right;
};

class KdTree {
  public:
    explicit KdTree();
    double dist(struct kd_node_t *a, struct kd_node_t *b, int dim);
    void swap(struct kd_node_t *x, struct kd_node_t *y);
    struct kd_node_t* find_median(struct kd_node_t *start, struct kd_node_t *end, int idx);
    struct kd_node_t* make_tree(struct kd_node_t *t, int len, int i, int dim);
    void nearest(struct kd_node_t *root, struct kd_node_t *nd, int i, int dim,
                 struct kd_node_t **best, double *best_dist);
    int visited;

  private:

};

#endif
