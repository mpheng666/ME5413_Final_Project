#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>

// Global Pointers
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_floor(new pcl::PointCloud<pcl::PointXYZ>);

// Parameters
std::string file_directory, file_name, map_topic_name;
double map_resolution, normal_threshold;
double search_radius; // For normal estimation

void VoxelGridFilter(double leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(pcd_cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*cloud_filtered);
    ROS_INFO("Voxel filter complete. Points: %lu", cloud_filtered->points.size());
}

void FilterHorizontalSurfaces(double threshold) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use a radius relative to resolution for stability
    ne.setRadiusSearch(map_resolution * 10.0); 
    ne.compute(*cloud_normals);

    cloud_no_floor->clear();
    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        // If normal_z is NaN (can happen with sparse points), skip it
        if (!std::isfinite(cloud_normals->points[i].normal_z)) continue;

        if (std::abs(cloud_normals->points[i].normal_z) < threshold) {
            cloud_no_floor->push_back(cloud_filtered->points[i]);
        }
    }
}
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid &msg) {
    if (cloud->points.empty()) {
        ROS_WARN("Filtered cloud is empty! Check your normal_threshold.");
        return;
    }

    float x_min = 1e10, x_max = -1e10, y_min = 1e10, y_max = -1e10;
    for (const auto& point : cloud->points) {
        x_min = std::min(x_min, point.x); x_max = std::max(x_max, point.x);
        y_min = std::min(y_min, point.y); y_max = std::max(y_max, point.y);
    }

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.info.resolution = map_resolution;
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = std::ceil((x_max - x_min) / map_resolution);
    msg.info.height = std::ceil((y_max - y_min) / map_resolution);
    
    msg.data.assign(msg.info.width * msg.info.height, 0);

    for (const auto& point : cloud->points) {
        int i = (point.x - x_min) / map_resolution;
        int j = (point.y - y_min) / map_resolution;
        if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
            msg.data[i + j * msg.info.width] = 100;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcd2pgm_multifloor");
    ros::NodeHandle nh("~");

    ros::Publisher intermediate_pub = nh.advertise<sensor_msgs::PointCloud2>("intermediate_cloud", 1, true);

    nh.param("file_directory", file_directory, std::string("/home/user/"));
    nh.param("file_name", file_name, std::string("map"));
    nh.param("map_resolution", map_resolution, 0.05);
    nh.param("normal_threshold", normal_threshold, 0.6); // 0.0=vertical, 1.0=horizontal
    nh.param("map_topic_name", map_topic_name, std::string("map"));

    std::string full_path = file_directory + file_name + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(full_path, *pcd_cloud) == -1) {
        ROS_ERROR("Failed to load %s", full_path.c_str());
        return -1;
    }

    // 1. Downsample (Reduces 26M to ~500k) - CRITICAL FOR SPEED
    VoxelGridFilter(map_resolution);

    // 2. Remove floors/ceilings via Normals (Works for multi-floor + ramps)
    FilterHorizontalSurfaces(normal_threshold);

    ROS_INFO("Final cloud after filtering: %lu points", cloud_no_floor->points.size());

    // Publish intermediate cloud for visualization/debugging
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_no_floor, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    intermediate_pub.publish(cloud_msg);

    ROS_INFO("Published intermediate cloud. Now converting to OccupancyGrid...");

    // 3. Convert to 2D Grid
    nav_msgs::OccupancyGrid map_msg;
    SetMapTopicMsg(cloud_no_floor, map_msg);

    ROS_INFO("Map conversion complete. Publishing on topic: %s", map_topic_name.c_str());

    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1, true);
    
    ros::Rate loop(1.0);
    while (ros::ok()) {
        map_msg.header.stamp = ros::Time::now();
        pub.publish(map_msg);
        intermediate_pub.publish(cloud_msg); // Keep publishing for visualization

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}