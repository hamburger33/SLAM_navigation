#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <pcl/filters/voxel_grid.h>  // 体素滤波

class MapLoader {
private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher cloud_pub_;  // 降采样后的点云发布

    // 参数变量
    std::string pcd_file_;
    int map_width_, map_height_;
    float resolution_;
    float height_threshold_;  // 高度过滤阈值

public:
    MapLoader() : nh_("~") {
        // 从参数服务器加载配置
        nh_.param<std::string>("pcd_file", pcd_file_, "/home/yaya/catkin_point_lio_unilidar/src/point_lio_unilidar/PCD/scans.pcd");
        nh_.param<int>("map_width", map_width_, 500);
        nh_.param<int>("map_height", map_height_, 500);
        nh_.param<float>("resolution", resolution_, 0.05);
        nh_.param<float>("height_threshold", height_threshold_, 0.1);  // 忽略高于此值的点

        // 初始化发布器
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true); // latch=true
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("map_markers", 1);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1);

        // 加载并处理点云
        loadAndProcessCloud();
    }

private:
    void loadAndProcessCloud() {
        // 加载原始点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud) == -1) {
            ROS_ERROR("Failed to load PCD file: %s", pcd_file_.c_str());
            return;
        }

        // 体素滤波降采样（新增）
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm体素大小
        voxel_filter.filter(*cloud);

        // 发布降采样后的点云（可选）
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_pub_.publish(cloud_msg);

        // 生成并发布地图
        nav_msgs::OccupancyGrid map;
        generateMap(cloud, map);
        map_pub_.publish(map);

        // 发布简化后的可视化标记（可选）
        publishMarkers(cloud);
    }

    void generateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, nav_msgs::OccupancyGrid& map) {
        // 设置地图元数据
        map.header.frame_id = "map";
        map.info.resolution = resolution_;
        map.info.width = map_width_;
        map.info.height = map_height_;
        map.info.origin.position.x = -map_width_ * resolution_ / 2.0;  // 中心为原点
        map.info.origin.position.y = -map_height_ * resolution_ / 2.0;
        map.info.origin.orientation.w = 1.0;
        map.data.resize(map_width_ * map_height_, -1);  // 初始化为未知

        // 遍历点云填充地图
        for (const auto& point : *cloud) {
            // 过滤高处点（如天花板）
            if (point.z > height_threshold_) continue;

            // 计算地图坐标索引
            int x = static_cast<int>((point.x - map.info.origin.position.x) / resolution_);
            int y = static_cast<int>((point.y - map.info.origin.position.y) / resolution_);

            if (x >=0 && x < map_width_ && y >=0 && y < map_height_) {
                map.data[y * map_width_ + x] = 100;  // 占用区域
            }
        }
    }

    void publishMarkers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;

        // 基础设置
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = 0.01;  // 薄片高度
        marker.color.a = 0.5;
        marker.color.r = 1.0;    // 红色表示障碍物
        marker.id = 0;

        // 填充点
        for (const auto& point : *cloud) {
            if (point.z > height_threshold_) continue;
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;  // 投影到地面
            marker.points.push_back(p);
        }

        markers.markers.push_back(marker);
        marker_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_loader");
    MapLoader loader;
    ros::spin();
    return 0;
}