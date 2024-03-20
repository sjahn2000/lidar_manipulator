#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarAccumulator {
public:
    LidarAccumulator() {
        accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_pointcloud", 1);
        scan_sub = nh.subscribe("scan", 1, &LidarAccumulator::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (!std::isinf(range)) {
                float angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXYZ point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0;
                cloud->points.push_back(point);
            }
        }

        *accumulated_cloud += *cloud;

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*accumulated_cloud, cloud_msg);
        cloud_msg.header.frame_id = "laser_frame";
        cloud_pub.publish(cloud_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_accumulator_node");

    LidarAccumulator accumulator;

    ros::spin();

    return 0;
}
