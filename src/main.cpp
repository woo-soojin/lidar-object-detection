/* \author Aaron Brown */
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

/**
 * Developer: Yasen Hu
 * Date: 05/25/2019
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("FOC GUI")); // TODO

ProcessPointClouds<pcl::PointXYZI> point_cloud_processor; // TODO input
//std::vector<boost::filesystem::path> stream = point_cloud_processor.streamPcd("../data/pcd/data_2"); // TODO remove
//auto stream_iterator = stream.begin(); // TODO remove
pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // TODO

pcl::visualization::PCLVisualizer::Ptr visualize_shape() // TODO name, return type
{
//    viewer->setBackgroundColor (0, 0, 0); // TODO remove

    pcl::ModelCoefficients coeffs;
    coeffs.values.clear ();
    coeffs.values.push_back (0);
    coeffs.values.push_back (0);
    coeffs.values.push_back (5);
    viewer->addCircle(coeffs, "circle"); // TODO name
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "circle");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "circle");

    coeffs.values.clear ();
    coeffs.values.push_back (0);
    coeffs.values.push_back (0);
    coeffs.values.push_back (10);
    viewer->addCircle(coeffs, "circle2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "circle2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,0, "circle2");

    coeffs.values.clear ();
    coeffs.values.push_back (0);
    coeffs.values.push_back (0);
    coeffs.values.push_back (15);
    viewer->addCircle(coeffs, "circle3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "circle3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "circle3");

    return (viewer); // TODO remove
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_cloud_processor, pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
    renderPointCloud(viewer, input_cloud, "InputCloud");

    // Input point cloud, filter resolution, min Point, max Point
    constexpr float kFilterResolution = 0.2;
    const Eigen::Vector4f kMinPoint(-50, -6.0, -3, 1);
    const Eigen::Vector4f kMaxPoint(60, 6.5, 4, 1);
    auto filter_cloud = point_cloud_processor.FilterCloud(input_cloud, kFilterResolution, kMinPoint, kMaxPoint);

//    renderPointCloud(viewer, filter_cloud, "FilteredCloud");

    constexpr int kMaxIterations = 100;
    constexpr float kDistanceThreshold = 0.2;
//    auto segment_cloud = point_cloud_processor.SegmentPlane(filter_cloud, kMaxIterations, kDistanceThreshold); // TODO remove

//    // render obstacles point cloud with red
//    renderPointCloud(viewer, segment_cloud.first, "ObstacleCloud", Color(1, 0, 0));
    // render ground plane with green
//    renderPointCloud(viewer, segment_cloud.second, "GroundCloud", Color(0, 1, 0));


    /*** Euclidean clustering ***/
    // float clusterTolerance, int minSize, int maxSize
    constexpr float kClusterTolerance = 0.35;
    constexpr int kMinSize = 15;
    constexpr int kMaxSize = 600;
//    auto cloud_clusters = point_cloud_processor.Clustering(segment_cloud.first, kClusterTolerance, kMinSize, kMaxSize); // TODO remove

    int cluster_ID = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};
    int num_of_colors = colors.size();

    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);

    constexpr float kBBoxMinHeight = 0.75;
//    for(const auto& cluster : cloud_clusters) { // TODO remove
    int num_of_bbox = 5; // TODO remove, Subscribe # of bbox
    for(int i=0; i<num_of_bbox; i++) { // TODO remove
//        renderPointCloud(viewer, cluster, "ObstacleCloud" + std::to_string(cluster_ID), colors[cluster_ID % num_of_colors]);

//        Box box = point_cloud_processor.BoundingBox(cluster); // TODO remove
        Box box; // TODO remove, Subscribe Bbox coordinates
        // Filter out some cluster with little points and shorter in height
//        if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) { // TODO remove
        if (box.z_max - box.z_min >= kBBoxMinHeight) { // TODO remove
            renderBox(viewer, box, cluster_ID);
        }

        cluster_ID++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    viewer->setBackgroundColor(0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle) {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem(1.0);
}

void run_pcl_viewer(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    visualize_shape();

    // Run obstacle detection process
    cityBlock(viewer, point_cloud_processor, input_cloud);

    // viewer spin
    viewer->spinOnce();
}

void callback(const sensor_msgs::PointCloud2& msg) {
    pcl::fromROSMsg(msg, *input_cloud);
    run_pcl_viewer(input_cloud);
    std::cout << input_cloud->size() << std::endl;
}

int main (int argc, char** argv) {
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    // Initialize ROS
    ros::init (argc, argv, "FOC_GUI");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/ouster/points", 1, callback); // TODO
//    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, callback); // TODO
    ros::spin();

    return 0;
}