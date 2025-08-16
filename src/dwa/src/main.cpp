#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include "dwa_demo/DWA.h"
#include "utils/visualization.h"

class DWALocalPlanner {
private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber rviz_goal_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher filtered_pointcloud_pub_;
    
    Eigen::VectorXd current_state_; // 状态量 [x, y, yaw, v, w]
    Eigen::Vector2d current_goal_;
    std::vector<Eigen::Vector2d> obstacles_; // centroids of obstacles
    

    DWA* dwa_planner_;
    
    double v_max_, w_max_, robot_radius_; // hyper parameters(limits) for DWA

    visualization_utils vis_utils_; // vis
    bool DEBUG_;
    bool RVIZ_GOAL_;
public:
    DWALocalPlanner() {
        
        DEBUG_ = nh_.param("DEBUG", false);
        RVIZ_GOAL_ = nh_.param("RVIZ_GOAL", true);
        odom_sub_ = nh_.subscribe("/odom", 100, &DWALocalPlanner::odomCallback, this);
        global_path_sub_ = nh_.subscribe("/global_path", 1, &DWALocalPlanner::globalPathCallback, this);
        pointcloud_sub_ = nh_.subscribe("/static_obstacles", 1, &DWALocalPlanner::pointCloudCallback, this);
        rviz_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DWALocalPlanner::getGoalFromRviz, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        filtered_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_obstacles", 1);

        current_state_ = Eigen::VectorXd::Zero(5);  // [x, y, yaw, v, w]
        
        loadParameters();
        
        // 创建DWA实例
        const double dt = 0.1, predict_time = 3.0, v_sample = 0.05, w_sample = 0.1;
        const double alpha = 0.8, beta = 0.1, gamma = 0.1;
        dwa_planner_ = new DWA(dt, 0, v_max_, -w_max_, w_max_, predict_time,
                              v_max_/2, w_max_/2, v_sample, w_sample,
                              alpha, beta, gamma, robot_radius_, 0.01);
    }

    void loadParameters() {
        nh_.param("max_vel", v_max_, 2.0);
        nh_.param("max_rot_vel", w_max_, 1.5);
        nh_.param("robot_radius", robot_radius_, 0.3);
    }

    /// @brief 将传感器坐标系下的点转换到全局坐标系
    /// @param point 传感器坐标系下的点
    /// @return 转换后的全局坐标系下的点
    Eigen::Vector2d transformToGlobal(const geometry_msgs::PointStamped& point) {
        geometry_msgs::PointStamped global_point;
        try {
            tf_listener_.transformPoint("odom", point, global_point); // 将点从传感器坐标系转换到map坐标系(odom)
            return {global_point.point.x, global_point.point.y};
        } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM("TF exception: " << ex.what());
            return Eigen::Vector2d::Zero();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 更新机器人状态 [x, y, yaw, v, w]
        current_state_(0) = msg->pose.pose.position.x;
        current_state_(1) = msg->pose.pose.position.y;
        if (DEBUG_)
        {
            ROS_INFO_STREAM("Current position: (" << current_state_(0) << ", " << current_state_(1) << ")");
        }
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_state_(2) = yaw;
        
        current_state_(3) = msg->twist.twist.linear.x;
        current_state_(4) = msg->twist.twist.angular.z;
    }

    /// @brief Get the global path and set the current goal
    /// @param msg 
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) return;
        
        // TODO: 选择最近的局部目标点 (优化: 选择预瞄距离的点)
        geometry_msgs::PointStamped point;
        point.header = msg->header;
        point.point = msg->poses.back().pose.position;
        
        if (!RVIZ_GOAL_)
        {
        current_goal_ = transformToGlobal(point); // Update current goal
        }

        ROS_INFO_STREAM("Current goal set to: " << current_goal_.transpose());
    }

    /// @brief Get 2d nav goal from rviz
    /// @param msg 
    void getGoalFromRviz(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double x=msg->pose.position.x;
        double y=msg->pose.position.y;
        //四元数转欧拉角
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.orientation, quat);
        double roll, pitch, yaw;//定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
        //打印输出
        ROS_INFO_STREAM("Get 2d nav goal:");
        ROS_INFO_STREAM("pos-x: "<<x<<"pos-y: "<<y<<"pos-yaw: "<<yaw);
        if (RVIZ_GOAL_)
        {
            current_goal_ = Eigen::Vector2d(x, y); // Update current goal
        }

    }

    /// @brief Main callback function
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        obstacles_.clear();
        
        // Convert sensor_msgs::PointCloud to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_cloud->header = cloud->header;
        
        // Process each point in the cloud
        for (const auto& point : cloud->points) {
            // Skip invalid points
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue; 
                
            // Filter points (optional: remove ground plane or points too far away)
            // if (point.z < -0.1 || point.z > 0.5)  // Adjust height filter as needed
            //     continue;
                
            // Convert point to map frame
            geometry_msgs::PointStamped pcl_point;
            pcl_point.header = cloud_msg->header;
            pcl_point.point.x = point.x;
            pcl_point.point.y = point.y;
            
            Eigen::Vector2d obs = transformToGlobal(pcl_point);
            if (!obs.isZero()) {
                obstacles_.push_back(obs);
                // ROS_INFO_STREAM("Obstacle detected at: " << obs.transpose());
                filtered_cloud->points.push_back(point);
            }
        }
        
        // Convert filtered point cloud to ROS message and publish
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_msg->header;
        filtered_pointcloud_pub_.publish(filtered_cloud_msg); //for rviz

        executeControlCycle();
    }

    void executeControlCycle() {
        // 调用DWA计算控制命令
        auto [control, trajectory] = dwa_planner_->dwaControl(current_state_, current_goal_, obstacles_);
        
        // 发布速度命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = control[0];  // 线速度
        cmd_vel.angular.z = control[1]; // 角速度
        cmd_vel_pub_.publish(cmd_vel);
        
        // 可视化轨迹 (调试用)
        vis_utils_.visualizeTrajectory(trajectory);
        vis_utils_.visualizeGoal(current_goal_);
        ROS_INFO_STREAM("Published cmd_vel: linear.x = " << cmd_vel.linear.x << ", angular.z = " << cmd_vel.angular.z);
    }


    ~DWALocalPlanner() {
        delete dwa_planner_;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dwa_local_planner");
    ROS_INFO("DWA Local Planner Node Started");
    DWALocalPlanner planner;
    ros::spin();
    return 0;
}