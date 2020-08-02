/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

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

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 100, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

}
