//
// Created by sen on 2020/08/08.
//

#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

#endif //PLAYBACK_CLUSTER_H

#include <unordered_set>
#include "kdtree.h"

void proximity(std::unordered_set<int>& processed_ids, const std::vector<std::vector<float>>& points,
               std::vector<int>& cluster_ids, int index, KdTree* tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree,
                                               float distanceTol);