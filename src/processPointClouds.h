// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <random>
#include <memory>
#include <unordered_set>
#include "render/box.h"
#include "cluster/kdtree.h"

// coefficients of ransac plane, used to record the plane model with the most inliers
struct Coefficient {
    float a;
    float b;
    float c;
    float d;
    Coefficient() : a(0.0f), b(0.0f), c(0.0f), d(0.0f) {}
    Coefficient(float a, float b, float c, float d) : a(a), b(b), c(c), d(d) {}
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds() = default;
    //deconstructor
    ~ProcessPointClouds() = default;

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                      float filterRes, Eigen::Vector4f minPoint,
                                                      Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(
            pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
            typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
            typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void SavePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr LoadPcd(std::string file);

    std::vector<boost::filesystem::path> StreamPcd(std::string dataPath);

    void Proximity(std::unordered_set<int>& processed_ids, const typename pcl::PointCloud<PointT>::Ptr& cloud,
                   std::vector<int>& cluster_ids, int index, KdTree<PointT>* tree, float distanceTol);

    std::vector<std::vector<int>> EuclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                                   KdTree<PointT>* tree,
                                                   float distanceTol,
                                                   int minSize,
                                                   int maxSize);

    // Ransac using CountWithinDistance and SelectWithinDistance.
    std::vector<int> Ransac(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                            int maxIterations,
                            float distanceTol);

    // I found that pcl separates countWithinDistance and selectWithinDistance for ransac.
    // https://pointclouds.org/documentation/sac__model__line_8hpp_source.html
    // I follow the same method so that I don't need to maintain an unordered_set for every iteration.
    std::pair<int, Coefficient> CountWithinDistance(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                                    float distanceTolerance);

    // Get indices of those points which are within distanceTolerance from the plane.
    // The plane is specified by coefficient.
    std::vector<int> SelectWithinDistance(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                          float distanceTolerance,
                                          const Coefficient& coefficient);

    // recursive function to sort left lists and right lists recursively.
    void BuildHelper(int depth,
                     const typename pcl::PointCloud<PointT>::Ptr& cloud,
                     std::vector<int>& indices,
                     int beginIndex,
                     int endIndex,
                     KdTree<PointT>* kdTree);
};
#endif /* PROCESSPOINTCLOUDS_H_ */