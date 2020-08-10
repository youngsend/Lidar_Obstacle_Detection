/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud){
    auto filtered_cloud = point_processor.FilterCloud(input_cloud, 0.1f,
                                                      Eigen::Vector4f(-10, -6, -2, 1),
                                                      Eigen::Vector4f(30, 7, 1, 1));
    // step 1: segment road and obstacles
    auto segment_cloud = point_processor.SegmentPlane(filtered_cloud, 100, 0.2);
    renderPointCloud(viewer, segment_cloud.second, "plane_cloud", Color(0,1,0));

    // step 2: cluster the obstacle cloud and find bounding boxes for them
    auto cloud_clusters = point_processor.Clustering(segment_cloud.first, 0.4, 40, 2000);
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0),
                                 Color(1,1,0),
                                 Color(0,0,1)};
    for(const auto& cloud_cluster : cloud_clusters){
        std::cout << "cluster size: ";
        point_processor.numPoints(cloud_cluster);
        renderPointCloud(viewer, cloud_cluster, "obstacle_cloud"+std::to_string(cluster_id),
                         colors[cluster_id%(colors.size())]);

        Box box = point_processor.BoundingBox(cloud_cluster);
        renderBox(viewer, box, cluster_id);

        ++cluster_id;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // pcl viewerを作る
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> point_processor;
    auto stream = point_processor.StreamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();

    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd and run obstacle detection process
        auto input_cloud = point_processor.LoadPcd((*stream_iterator).string());
        cityBlock(viewer, point_processor, input_cloud);

        stream_iterator++;
        if(stream_iterator == stream.end()) {
            stream_iterator = stream.begin();
        }

        viewer->spinOnce();
    }
}