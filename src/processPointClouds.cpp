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

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(const auto& cluster_index : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(auto index : cluster_index.indices) {
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

    Box box;
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

        float gain = 1.0 / std::sqrt(a*a + b*b + c*c);
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
