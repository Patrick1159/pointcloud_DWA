#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

class visualization_utils
{
private:
    /* data */
public:
    visualization_utils(/* args */);
    ~visualization_utils();

    /// @brief 轨迹可视化 (RViz marker)
    /// @param trajectory 轨迹点集
    /// @param id 轨迹ID，用于区分不同轨迹
    /// @param r 红色分量 (0-1)
    /// @param g 绿色分量 (0-1)
    /// @param b 蓝色分量 (0-1)
    void visualizeTrajectory(const std::vector<Eigen::VectorXd>& trajectory, 
                             int id = 0, 
                             float r = 1.0, float g = 0.0, float b = 0.0) {
        // Create a publisher for trajectory visualization if not done already
        static ros::NodeHandle vis_nh_;
        static ros::Publisher marker_pub = vis_nh_.advertise<visualization_msgs::Marker>("/trajectory_marker", 10);

        // Create a marker message
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;  // Line width
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        for (const auto& state : trajectory) {
            geometry_msgs::Point p;
            p.x = state(0);
            p.y = state(1);
            p.z = 0.1;
            marker.points.push_back(p);
        }

        if (!marker.points.empty()) {
            marker_pub.publish(marker);
        }
    }


    /// @brief 目标点可视化 (RViz marker)
    /// @param goal 目标点
    void visualizeGoal(const Eigen::Vector2d& goal) {
        // Create a publisher for goal visualization if not done already
        static ros::NodeHandle vis_nh_;
        static ros::Publisher goal_marker_pub = vis_nh_.advertise<visualization_msgs::Marker>("/goal_marker", 1);

        // Create a marker message
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Set the position
        marker.pose.position.x = goal(0);
        marker.pose.position.y = goal(1);
        marker.pose.position.z = 0.2;  // Slightly above ground for visibility
        marker.pose.orientation.w = 1.0;
        
        // Set the scale (size of the sphere)
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        // Set the color (green)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // Set marker lifetime
        marker.lifetime = ros::Duration();
        
        // Publish the marker
        goal_marker_pub.publish(marker);
    }
};

visualization_utils::visualization_utils(/* args */)
{
}

visualization_utils::~visualization_utils()
{
}

// /// @brief 轨迹可视化 (RViz marker)
// /// @param trajectory 
// void visualizeTrajectory(const std::vector<Eigen::VectorXd>& trajectory) {
//         // Create a publisher for trajectory visualization if not done already
//         static ros::NodeHandle vis_nh_;
//         static ros::Publisher marker_pub = vis_nh_.advertise<visualization_msgs::Marker>("/trajectory_marker", 1);

//         // Create a marker message
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "base_link";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "trajectory";
//         marker.id = 0;
//         marker.type = visualization_msgs::Marker::LINE_STRIP;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.orientation.w = 1.0;
//         marker.scale.x = 0.05;  // Line width
//         marker.color.r = 1.0;
//         marker.color.g = 0.0;
//         marker.color.b = 0.0;
//         marker.color.a = 1.0;

//         // Add trajectory points to marker
//         for (const auto& state : trajectory) {
//             geometry_msgs::Point p;
//             p.x = state(0);
//             p.y = state(1);
//             p.z = 0.1;  // Slightly above ground for visibility
//             marker.points.push_back(p);
//         }

//         // Only publish if there are points
//         if (!marker.points.empty()) {
//             marker_pub.publish(marker);
//         }
//     }


// /// @brief 目标点可视化 (RViz marker)
// /// @param goal 目标点
// void visualizeGoal(const Eigen::Vector2d& goal) {
//     // Create a publisher for goal visualization if not done already
//     static ros::NodeHandle vis_nh_;
//     static ros::Publisher goal_marker_pub = vis_nh_.advertise<visualization_msgs::Marker>("/goal_marker", 1);

//     // Create a marker message
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "base_link";
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "goal";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::SPHERE;
//     marker.action = visualization_msgs::Marker::ADD;
    
//     // Set the position
//     marker.pose.position.x = goal(0);
//     marker.pose.position.y = goal(1);
//     marker.pose.position.z = 0.2;  // Slightly above ground for visibility
//     marker.pose.orientation.w = 1.0;
    
//     // Set the scale (size of the sphere)
//     marker.scale.x = 0.3;
//     marker.scale.y = 0.3;
//     marker.scale.z = 0.3;
    
//     // Set the color (green)
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.color.a = 1.0;
    
//     // Set marker lifetime
//     marker.lifetime = ros::Duration();
    
//     // Publish the marker
//     goal_marker_pub.publish(marker);
// }