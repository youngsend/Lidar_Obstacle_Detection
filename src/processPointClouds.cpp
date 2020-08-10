// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud){
    std::cout << cloud->points.size() << std::endl;
}

/**
 * Filter out points in area where we are not interested.
 * For example, the roof of ego car, the walls outside the road.
 * And only
 * @tparam PointT
 * @param cloud
 * @param filterRes
 * @param minPoint
 * @param maxPoint
 * @return
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint,
        Eigen::Vector4f maxPoint){
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in the function to do voxel grid point reduction and region based filtering
    // Voxel grid filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    sor.filter(*cloud_filtered);

    // crop region of interest
    pcl::CropBox<PointT> cropBox;
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>());
    cropBox.filter(*cloud_cropped);

    // filter out the roof points
    pcl::CropBox<PointT> roof;
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_cropped);
    std::vector<int> indices;
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_cropped);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped;
}

/**
 * Separate one cloud into two using two groups of indices.
 * @tparam PointT
 * @param inliers
 * @param cloud
 * @return  pair<obstacle point cloud, plane point cloud>.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud) {
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloudPlane (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudObst (new pcl::PointCloud<PointT>());

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliners
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // false (positive) when extracting plane cloud. 直接にinliersのindicesでplane pointsを取ってもいい。
    extract.setNegative(false);
    extract.filter(*cloudPlane);

    // true (negative) when extracting other cloud
    extract.setNegative(true);
    extract.filter(*cloudObst);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(
            cloudObst, cloudPlane);
    return segResult;
}

/**
 * Segment road plane from other obstacles.
 * Obstacle cloud will be clustered further.
 * @tparam PointT
 * @param cloud
 * @param maxIterations
 * @param distanceThreshold
 * @return
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                         float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // insert unordered_set into vector
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    // measure ransac time
    inliers->indices = Ransac(cloud, maxIterations, distanceThreshold);

    if (inliers->indices.empty()){
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult =
            SeparateClouds(inliers,cloud);
    return segResult;
}

/**
 * Use cloud points to build a kd-tree and find clusters.
 * For each cluster, create a point cloud.
 * @tparam PointT
 * @param cloud
 * @param clusterTolerance
 * @param minSize
 * @param maxSize
 * @return
 */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // construction of kd-tree
    KdTree<PointT>* tree(new KdTree<PointT>());

    // insert point in cloud into kd-tree
    std::vector<int> indices(cloud->points.size());
    std::iota(indices.begin(), indices.end(), 0);
    BuildHelper(0, cloud, indices, 0, cloud->points.size()-1, tree);

    // cluster cloud by searching kd-tree
    auto clusters_indices = EuclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

    // create point cloud for each cluster.
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for(const auto& cluster_indices : clusters_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(auto index : cluster_indices) {
            cloud_cluster->push_back(cloud->points[index]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster){
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::SavePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file){
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::LoadPcd(std::string file){
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::StreamPcd(std::string dataPath){
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

/**
 * Call CountWithinDistance for maxIterations times, and record the plane model which have the most inliers.
 * For the final best plane model, get a vector of cloud indices for its inliers.
 * @tparam PointT
 * @param cloud
 * @param maxIterations
 * @param distanceTol
 * @return
 */
template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::Ransac(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                                    int maxIterations, float distanceTol) {
    int max_count = -1;
    Coefficient coefficient_best_plane;

    // find the best plane's coefficients.
    while (maxIterations--) {
        auto result = CountWithinDistance(cloud, distanceTol);
        // update inliersResult if more inliers recorded.
        if (result.first > max_count) {
            coefficient_best_plane = result.second;

            // update max_count!
            max_count = result.first;
        }
    }

    // use the best plane model to collect road plane indices.
    return SelectWithinDistance(cloud, distanceTol, coefficient_best_plane);
}

/**
 * Like a queue. For each point in the queue, find all points which are within distance tolerance.
 * If the found points do not exist in the queue, insert them into queue.
 * @tparam PointT
 * @param processed_ids
 * @param cloud
 * @param cluster_ids
 * @param index
 * @param tree
 * @param distanceTol
 */
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(std::unordered_set<int> &processed_ids,
                                           const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                           std::vector<int> &cluster_ids,
                                           int index, KdTree<PointT> *tree, float distanceTol) {
    processed_ids.insert(index);
    cluster_ids.push_back(index);
    auto nearby_points = tree->search(cloud->points[index], distanceTol);
    for (int nearby_index : nearby_points) {
        if (!processed_ids.count(nearby_index)) {
            Proximity(processed_ids, cloud, cluster_ids, nearby_index, tree, distanceTol);
        }
    }
}

/**
 * Find all clusters in cloud whose size is between minSize and maxSize.
 * Use Proximity function to collect points that belong to one cluster.
 * After one Proximity execution, some points are marked as processed.
 * For unprocessed points, new cluster will be initialized and filled with proximity function.
 * @tparam PointT
 * @param cloud
 * @param tree
 * @param distanceTol
 * @param minSize
 * @param maxSize
 * @return
 */
template <typename PointT>
std::vector<std::vector<int>>
ProcessPointClouds<PointT>::EuclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                             KdTree<PointT>* tree,
                                             float distanceTol,
                                             int minSize,
                                             int maxSize){
    // Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processed_ids;
    for (int index=0; index<cloud->points.size(); index++){
        if(!processed_ids.count(index)) {
            std::vector<int> cluster_ids;
            Proximity(processed_ids, cloud, cluster_ids, index, tree, distanceTol);
            if (minSize <= cluster_ids.size() && cluster_ids.size() <= maxSize) {
                // add minSize and maxSize constraint.
                clusters.push_back(cluster_ids);
            }
        }
    }
    return clusters;
}

/**
 * Randomly select 3 points to form a plane model and count those points which are within distanceTolerance from
 * the plane.
 * @tparam PointT
 * @param cloud
 * @param distanceTolerance
 * @return
 */
template <typename PointT>
std::pair<int, Coefficient> ProcessPointClouds<PointT>::CountWithinDistance(
        const typename pcl::PointCloud<PointT>::Ptr& cloud,
        float distanceTolerance){
    int count = 0;
    // Will be used to obtain a seed for the random number engine
    std::random_device rd;
    // Standard mersenne_twister_engine seeded with rd()
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, cloud->size()-1);

    // use set to prevent same points.
    std::set<int> tmpInliersResult;
    // Get 3 different point indices.
    while (tmpInliersResult.size() < 3) {
        tmpInliersResult.insert(distrib(gen));
    }

    // Get the 3 selected points.
    auto itr = tmpInliersResult.begin();
    auto x1 = cloud->points[*itr].x;
    auto y1 = cloud->points[*itr].y;
    auto z1 = cloud->points[*itr].z;

    itr++;
    auto x2 = cloud->points[*itr].x;
    auto y2 = cloud->points[*itr].y;
    auto z2 = cloud->points[*itr].z;

    itr++;
    auto x3 = cloud->points[*itr].x;
    auto y3 = cloud->points[*itr].y;
    auto z3 = cloud->points[*itr].z;

    // Calculate coefficients of plane
    float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

    float a = i;
    float b = j;
    float c = k;
    float d = -(i * x1 + j * y1 + k * z1);

    float gain = 1.0f / std::sqrt(a*a + b*b + c*c);
    for (int point_index = 0; point_index < cloud->size(); point_index++) {
        auto x4 = cloud->points[point_index].x;
        auto y4 = cloud->points[point_index].y;
        auto z4 = cloud->points[point_index].z;
        // use fabs!
        float distance = std::fabs(a*x4 + b*y4 + c*z4 + d) * gain;
        if (distance <= distanceTolerance) {
            count++;
        }
    }

    return std::make_pair(count, Coefficient(a, b, c, d));
}

template <typename PointT>
std::vector<int> ProcessPointClouds<PointT>::SelectWithinDistance(
        const typename pcl::PointCloud<PointT>::Ptr& cloud,
        float distanceTolerance,
        const Coefficient& coefficient){
    std::vector<int> indices;

    float a = coefficient.a;
    float b = coefficient.b;
    float c = coefficient.c;
    float d = coefficient.d;

    float gain = 1.0f / std::sqrt(a*a + b*b + c*c);
    for (int point_index = 0; point_index < cloud->size(); point_index++) {
        auto x4 = cloud->points[point_index].x;
        auto y4 = cloud->points[point_index].y;
        auto z4 = cloud->points[point_index].z;
        // use fabs!
        float distance = std::fabs(a*x4 + b*y4 + c*z4 + d) * gain;
        if (distance <= distanceTolerance) {
            indices.push_back(point_index);
        }
    }

    return indices;
}

/**
 * Help to sort vector and insert the median and recursively process left list and right list.
 * Important to check two things. First, whether the tree size equals to cloud size.
 * Second, whether points in cloud are inserted uniquely into tree.
 * @tparam PointT
 * @param depth
 * @param cloud
 * @param indices
 * @param beginIndex
 * @param endIndex
 * @param kdTree
 */
template <typename PointT>
void ProcessPointClouds<PointT>::BuildHelper(int depth,
                                             const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                             std::vector<int>& indices,
                                             int beginIndex,
                                             int endIndex,
                                             KdTree<PointT>* kdTree){
    if (endIndex < beginIndex)
        return;
    else if(endIndex == beginIndex) {
        kdTree->insert(cloud->points[indices[beginIndex]], indices[beginIndex]);
    } else {
        // when size of vector is larger or equal to 2, sort it by depth%3
        std::sort(indices.begin()+beginIndex, indices.begin()+endIndex+1, [&](int a, int b){
            return (cloud->points[a]).data[depth%3] < (cloud->points[b]).data[depth%3];
        });
        int median_index = beginIndex + (endIndex-beginIndex+1)/2;
        int median_value = indices[median_index];

        // insert the median point sorted by depth%3
        kdTree->insert(cloud->points[median_value], median_value);

        // process the left half and right half respectively.
        // sort does not include end iterator, so pass medianIt rather than medianIt-1.
        BuildHelper(depth+1, cloud, indices, beginIndex, median_index-1, kdTree);
        BuildHelper(depth+1, cloud, indices, median_index+1, endIndex, kdTree);
    }
}