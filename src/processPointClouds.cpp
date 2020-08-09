// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud){
    std::cout << cloud->points.size() << std::endl;
}

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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                         float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // Fill in this function to find inliers for the cloud.
    auto inlier_id_set = Ransac(cloud, maxIterations, distanceThreshold);

    // insert unordered_set into vector
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    (inliers->indices).insert((inliers->indices).end(), inlier_id_set.begin(), inlier_id_set.end());

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

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    // construction of kd-tree
    KdTree<PointT>* tree = new KdTree<PointT>();

    // decide the insert order.
    auto insert_order = GetInsertOrder(cloud);

    // insert point in cloud into kd-tree
    for(auto id : insert_order){
        tree->insert(cloud->points[id], id);
    }

    // cluster cloud by searching kd-tree
    auto clusters_indices = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file){
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file){
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath){
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template<typename PointT>
std::unordered_set<int>
ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;

    // Will be used to obtain a seed for the random number engine
    std::random_device rd;
    // Standard mersenne_twister_engine seeded with rd()
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, cloud->size()-1);

    // For max iterations
    // Randomly sample subset and fit line
    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    // Return indicies of inliers from fitted line with most inliers
    while (maxIterations--){
        // temporary result set
        std::unordered_set<int> tmpInliersResult;
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
        float i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

        float a = i;
        float b = j;
        float c = k;
        float d = -(i*x1 + j*y1 + k*z1);

        float gain = 1.0f / std::sqrt(a*a + b*b + c*c);
        for (int point_index = 0; point_index < cloud->size(); point_index++) {
            if(tmpInliersResult.count(point_index)) {
                continue;
            }

            auto point = cloud->points[point_index];
            // use fabs!
            float distance = std::fabs(a * point.x + b * point.y + c * point.z + d) * gain;
            if (distance <= distanceTol) {
                tmpInliersResult.insert(point_index);
            }
        }

        // update inliersResult if more inliers recorded.
        if (tmpInliersResult.size() > inliersResult.size()) {
            inliersResult = tmpInliersResult;
        }
    }

    return inliersResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(std::unordered_set<int> &processed_ids,
                                           const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                           std::vector<int> &cluster_ids,
                                           int index, KdTree<PointT> *tree, float distanceTol) {
    processed_ids.insert(index);
    cluster_ids.push_back(index);
    auto nearby_points = tree->search(cloud->points[index], distanceTol);
    for (int nearby_index : nearby_points) {
        if (!processed_ids.count(nearby_index)) {
            proximity(processed_ids, cloud, cluster_ids, nearby_index, tree, distanceTol);
        }
    }
}

template <typename PointT>
std::vector<std::vector<int>>
ProcessPointClouds<PointT>::euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud,
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
            proximity(processed_ids, cloud, cluster_ids, index, tree, distanceTol);
            if (minSize <= cluster_ids.size() && cluster_ids.size() <= maxSize) {
                // add minSize and maxSize constraint.
                clusters.push_back(cluster_ids);
            }
        }
    }
    return clusters;
}

static int GetMedian(const std::vector<bool>& processed, const std::vector<int>& order, int median_index){
    int count = 0;
    for(auto cloud_point_index : order){
        if(!processed[cloud_point_index]) {
            count++;
            // if median_index is 0, then only find 1 which has not been processed.
            if (count == median_index+1) {
                return cloud_point_index;
            }
        }
    }
    return -1;
}

template <typename PointT>
std::vector<int> ProcessPointClouds<PointT>::GetInsertOrder(const typename pcl::PointCloud<PointT>::Ptr& cloud){
    auto& points = cloud->points;

    std::vector<bool> processed(points.size(), false);
    std::vector<int> x_order(points.size());
    std::iota(x_order.begin(), x_order.end(), 0);
    std::vector<int> y_order(points.size());
    std::iota(y_order.begin(), y_order.end(), 0);
    std::vector<int> z_order(points.size());
    std::iota(z_order.begin(), z_order.end(), 0);

    // sort by x, y and z respectively.
    std::sort(x_order.begin(), x_order.end(), [&](int a, int b){
        return points[a].x < points[b].x;
    });
    std::sort(y_order.begin(), y_order.end(), [&](int a, int b){
        return points[a].y < points[b].y;
    });
    std::sort(z_order.begin(), z_order.end(), [&](int a, int b){
        return points[a].z < points[b].z;
    });

    // get x_order, y_order, z_order's median again and again
    int number_inserted = 0;
    int dimension = 3;
    std::vector<int> insert_order;
    while (number_inserted < points.size()){
        int median_index = (points.size() - number_inserted) / 2;
        int mod_index = number_inserted % dimension;
        int median = -1;
        if (mod_index==0) {
            // time to insert x median
            median = GetMedian(processed, x_order, median_index);
        } else if(mod_index == 1) {
            median = GetMedian(processed, y_order, median_index);
        } else {
            median = GetMedian(processed, z_order, median_index);
        }
        if (median == -1) {
            std::cerr << "Median not found! median_index: " << median_index << std::endl;
            break;
        }
        insert_order.push_back(median);
        processed[median] = true;
        number_inserted++;
    }

    return insert_order;
}