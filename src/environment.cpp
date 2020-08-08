/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2),
                Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2),
              Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2),
              Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2),
              Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor (on heap)
    auto lidar = new Lidar(cars, 0.0);
    auto inputCloud = lidar->scan();
    // lidar has position (0, 0, 2.6)!
//    renderRays(viewer, lidar->position, inputCloud);
//    renderPointCloud(viewer, inputCloud, "inputCloud");
    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    // Separate obstacle cloud and road cloud
    auto segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
//    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Clustering obstacle cloud
    auto cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0),
                                 Color(0,0,1)};

    for(const auto& cluster : cloudClusters){
        std::cout << "cluster size";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud){
    auto filtered_cloud = point_processor.FilterCloud(input_cloud, 0.1f,
                                                      Eigen::Vector4f(-10, -6, -2, 1),
                                                      Eigen::Vector4f(30, 7, 1, 1));
//    renderPointCloud(viewer, filtered_cloud, "filtered_cloud");
    // step 1: segment road and obstacles
    auto segment_cloud = point_processor.SegmentPlane(filtered_cloud, 100, 0.2);
//    renderPointCloud(viewer, segment_cloud.first, "obstacle_cloud", Color(1,0,0));
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
    auto stream = point_processor.streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();

    while (!viewer->wasStopped ())
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd and run obstacle detection process
        auto input_cloud = point_processor.loadPcd((*stream_iterator).string());
        cityBlock(viewer, point_processor, input_cloud);

        stream_iterator++;
        if(stream_iterator == stream.end()) {
            stream_iterator = stream.begin();
        }

        viewer->spinOnce();
    }
}