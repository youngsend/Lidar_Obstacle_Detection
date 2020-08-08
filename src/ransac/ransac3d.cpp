/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include <random>

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
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
