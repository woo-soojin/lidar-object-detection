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

#include "lidar_object_detection/bbox.h"
#include "render/render.h"

lidar_object_detection::bbox bounding_box; // TODO
pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // TODO

pcl::visualization::PCLVisualizer::Ptr visualize_shape(pcl::visualization::PCLVisualizer::Ptr viewer) // TODO name, return type, input type
{
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
    renderPointCloud(viewer, input_cloud, "InputCloud");

    // Input point cloud, filter resolution, min Point, max Point
    constexpr float kFilterResolution = 0.2;
    const Eigen::Vector4f kMinPoint(-50, -6.0, -3, 1);
    const Eigen::Vector4f kMaxPoint(60, 6.5, 4, 1);

    constexpr int kMinSize = 15;
    constexpr int kMaxSize = 600;

    int ID = 1;
    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);

    constexpr float kBBoxMinHeight = 0.75;
    Box box; // TODO remove, Subscribe Bbox coordinates
    int num_of_bbox = bounding_box.x_max.size(); // TODO
    for(int i=0; i<num_of_bbox; i++) { // TODO method
        box.x_min = bounding_box.x_min[i];
        box.y_min = bounding_box.y_min[i];
        box.z_min = bounding_box.z_min[i];
        box.x_max = bounding_box.x_max[i];
        box.y_max = bounding_box.y_max[i];
        box.z_max = bounding_box.z_max[i];

        renderBox(viewer, box, ID, Color(0.5, 0, 1));
        ID++;
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
    initCamera(setAngle, viewer);

    while(!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        visualize_shape(viewer);

        // Run obstacle detection process
        cityBlock(viewer, input_cloud);

        // viewer spin
        viewer->spinOnce(100);
    }
}

void callback1(const sensor_msgs::PointCloud2& msg) {
    pcl::fromROSMsg(msg, *input_cloud);
}

void callback2(lidar_object_detection::bbox msg) {
    bounding_box = msg;
}

int main (int argc, char** argv) {
    std::thread pclViewer(run_pcl_viewer, input_cloud);

    // Initialize ROS
    ros::init (argc, argv, "FOC_GUI");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/ouster/points", 1, callback1); // TODO
//    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, callback1); // TODO
    ros::Subscriber sub2 = nh.subscribe("/detector", 10, callback2); // TODO
    ros::spin();

    return 0;
}