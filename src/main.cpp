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
#include <thread>

#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // TODO

void drawRangeCircle(pcl::visualization::PCLVisualizer::Ptr viewer, float range1, float range2, float range3)
{
    pcl::ModelCoefficients coeffs;
    coeffs.values.clear ();
    coeffs.values.push_back (0);
    coeffs.values.push_back (0);
    coeffs.values.push_back (range1);
    viewer->addCircle(coeffs, "circle");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "circle");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "circle");

    coeffs.values.clear ();
    coeffs.values.push_back (0);
    coeffs.values.push_back (0);
    coeffs.values.push_back (range2);
    viewer->addCircle(coeffs, "circle2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "circle2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,0, "circle2");

    coeffs.values.clear ();
    coeffs.values.push_back (0);
    coeffs.values.push_back (0);
    coeffs.values.push_back (range3);
    viewer->addCircle(coeffs, "circle3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3, "circle3");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "circle3");
}

void drawRangeRectangle(pcl::visualization::PCLVisualizer::Ptr& viewer, float range1, float range2, float range3) {
    Box rectangle1 = {-range1, -range1, 0.0, range1, range1, 0.0};
    drawRectangle(viewer, rectangle1, "rectangle1", Color(1.0, 0.0, 0.0), 1.0);

    Box rectangle2 = {-range2, -range2, 0.0, range2, range2, 0.0};
    drawRectangle(viewer, rectangle2, "rectangle2", Color(1.0, 1.0, 0.0), 1.0);

    Box rectangle3 = {-range3, -range3, 0.0, range3, range3, 0.0};
    drawRectangle(viewer, rectangle3, "rectangle3", Color(0.0, 1.0, 0.0), 1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_cloud_processor, pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
    renderPointCloud(viewer, input_cloud, "InputCloud",  Color(0.3, 0.3, 0.3));

    // Input point cloud, filter resolution, min Point, max Point
    constexpr float kFilterResolution = 0.2;
    const Eigen::Vector4f kMinPoint(-50, -6.0, -3, 1);
    const Eigen::Vector4f kMaxPoint(60, 6.5, 4, 1);
    auto filter_cloud = point_cloud_processor.FilterCloud(input_cloud, kFilterResolution, kMinPoint, kMaxPoint);

    constexpr int kMaxIterations = 100;
    constexpr float kDistanceThreshold = 0.2;
    auto segment_cloud = point_cloud_processor.SegmentPlane(filter_cloud, kMaxIterations, kDistanceThreshold);

    constexpr float kClusterTolerance = 0.35;
    constexpr int kMinSize = 15;
    constexpr int kMaxSize = 600;
    auto cloud_clusters = point_cloud_processor.Clustering(segment_cloud.first, kClusterTolerance, kMinSize, kMaxSize);

    int cluster_ID = 2;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};
    int num_of_colors = colors.size();

    Box transporter = {-10.0, -3.5, -1, 10.0, 3.5, -0.4};
    renderBox(viewer, transporter, 0, Color(1.0, 0.5, 0.0), 1.0);

    Box block = {-12.5, -12.5, -0.4, 12.5, 12.5, 17.6};
    renderBox(viewer, block, 1, Color(0.8, 0.8, 0.8), 1.0);

    // range
    drawRangeRectangle(viewer, 15.0, 25.0, 35.0); // 30, 50, 70

    constexpr float kBBoxMinHeight = 0.75;
    for(const auto& cluster : cloud_clusters) {
        std::cout << "cluster size ";
        point_cloud_processor.numPoints(cluster);

        Box box = point_cloud_processor.BoundingBox(cluster);
        // Filter out some cluster with little points and shorter in height
        if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) {
            renderBox(viewer, box, cluster_ID, Color(1.0, 0.0, 0.0), 1.0);
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
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("FOC GUI")); // TODO
    CameraAngle setAngle = FPS;
//    initCamera(setAngle, viewer);
    int distance = 200;
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);

    ProcessPointClouds<pcl::PointXYZI> point_cloud_processor;

    while(!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Run obstacle detection process
        cityBlock(viewer, point_cloud_processor, input_cloud);

        // viewer spin
        viewer->spinOnce(100);
    }
}

void callback(const sensor_msgs::PointCloud2& msg) {
    pcl::fromROSMsg(msg, *input_cloud);
}

int main (int argc, char** argv) {
    std::thread pclViewer(run_pcl_viewer, input_cloud);

    // Initialize ROS
    ros::init (argc, argv, "FOC_GUI");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
//    ros::Subscriber sub = nh.subscribe("/ouster/points", 1, callback);
//    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, callback);
    ros::Subscriber sub = nh.subscribe("/lidar_total", 1, callback);
    ros::spin();

    return 0;
}