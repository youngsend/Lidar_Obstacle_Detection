/* \author Aaron Brown */
#include "cluster.h"

void proximity(std::unordered_set<int>& processed_ids, const std::vector<std::vector<float>>& points,
               std::vector<int>& cluster_ids, int index, KdTree* tree, float distanceTol) {
    processed_ids.insert(index);
    cluster_ids.push_back(index);
    auto nearby_points = tree->search(points[index], distanceTol);
    for (int nearby_index : nearby_points) {
        if (!processed_ids.count(nearby_index)) {
            proximity(processed_ids, points, cluster_ids, nearby_index, tree, distanceTol);
        }
    }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree,
                                               float distanceTol){
    // Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processed_ids;
    for (int index=0; index<points.size(); index++){
        if(!processed_ids.count(index)) {
            std::vector<int> cluster_ids;
            proximity(processed_ids, points, cluster_ids, index, tree, distanceTol);
            clusters.push_back(cluster_ids);
        }
    }
    return clusters;
}
