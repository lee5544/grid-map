/*
 * @Name:
 * @Author:       yong
 * @Date: 2023-03-11 14:40:26
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-10-22 16:31:36
 * @Description:
 * @Subscriber:
 * @Publisher:
 */
#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include "InflationMap.hpp"

struct cameraData
{
    /* depth image process */
    double cx, cy, fx, fy;
    int depth_width, depth_heigth;

    double depth_maxdist, depth_mindist;
    int depth_filter_margin;
    double k_depth_scaling_factor;
    int skip_pixel;

    Eigen::Vector3d camera_pos;
    Eigen::Quaterniond camera_q;

    Eigen::Matrix3d R_C_2_W, R_C_2_B;
    Eigen::Vector3d T_C_2_B, T_C_2_W;

    cv::Mat depth_image;
    pcl::PointCloud<pcl::PointXYZ> ptws_hit, ptws_miss;
};

cameraData camData_;

InflationMap map_;

ros::Timer map_timer_;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyImageOdom;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
SynchronizerImageOdom sync_image_odom_;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

// for visualization
ros::Publisher slide_global_map_range_pub_, local_update_range_pub_; // the sliding window size
ros::Publisher occ_pub_, inf_pub_;                                   // the sliding window size
ros::Publisher new_occ_pub_, new_free_pub_;

bool depth_need_update_;
Eigen::Vector3d local_map_boundary_min_, local_map_boundary_max_;

// visualization
void publishLocalUpdateRange()
{
    Eigen::Vector3d map_min_pos, map_max_pos, cube_pos, cube_scale;
    visualization_msgs::Marker mk;

    map_min_pos = local_map_boundary_min_;
    map_max_pos = local_map_boundary_max_;

    cube_pos = 0.5 * (map_min_pos + map_max_pos);
    cube_scale = map_max_pos - map_min_pos;

    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;

    mk.pose.position.x = cube_pos(0);
    mk.pose.position.y = cube_pos(1);
    mk.pose.position.z = cube_pos(2);

    mk.scale.x = cube_scale(0);
    mk.scale.y = cube_scale(1);
    mk.scale.z = cube_scale(2);

    mk.color.a = 0.2;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;

    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    local_update_range_pub_.publish(mk);
}

void publishMapRange()
{
    Eigen::Vector3d map_min_pos, map_max_pos, cube_pos, cube_scale;
    visualization_msgs::Marker mk;

    map_min_pos = map_.SOGMPtr_->VoxelToWorld(map_.SOGMPtr_->getOrigin());
    map_max_pos = map_.SOGMPtr_->VoxelToWorld(map_.SOGMPtr_->getOrigin()) + map_.SOGMPtr_->VoxelToWorld(map_.SOGMPtr_->getNum3dim());
    // map_min_pos = local_map_boundary_min_;
    // map_max_pos = local_map_boundary_max_;

    cube_pos = 0.5 * (map_min_pos + map_max_pos);
    cube_scale = map_max_pos - map_min_pos;

    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;

    mk.pose.position.x = cube_pos(0);
    mk.pose.position.y = cube_pos(1);
    mk.pose.position.z = cube_pos(2);

    mk.scale.x = cube_scale(0);
    mk.scale.y = cube_scale(1);
    mk.scale.z = cube_scale(2);

    mk.color.a = 0.2;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    slide_global_map_range_pub_.publish(mk);
}

void publishNewOcc()
{
    std::vector<int> *newOcc;
    newOcc = map_.SOGMPtr_->getNewOcc();

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;

    for (size_t i = 0; i < newOcc->size(); i++)
    {
        pos = map_.SOGMPtr_->IndexToWorld(newOcc->at(i));

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    new_occ_pub_.publish(cloud_msg);
}

void publishNewFree()
{
    std::vector<int> *newFree;
    newFree = map_.SOGMPtr_->getNewFree();

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;

    for (size_t i = 0; i < newFree->size(); i++)
    {
        pos = map_.SOGMPtr_->IndexToWorld(newFree->at(i));

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    new_free_pub_.publish(cloud_msg);
}

void publishOccMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;
    int index;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = map_.SOGMPtr_->getOrigin();
    max_cut = map_.SOGMPtr_->getOrigin() + map_.SOGMPtr_->getNum3dim();

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z)
            {
                index = map_.SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, z));
                if (!map_.SOGMPtr_->isOccupied(index))
                    continue;

                pos = map_.SOGMPtr_->IndexToWorld(index);

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.points.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    occ_pub_.publish(cloud_msg);
}

void publishInfMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;
    int index;

    Eigen::Vector3i min_cut, max_cut;
    min_cut = map_.SOGMPtr_->getOrigin();
    max_cut = map_.SOGMPtr_->getOrigin() + map_.SOGMPtr_->getNum3dim();

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z)
            {
                index = map_.SOGMPtr_->VoxelToIndex(Eigen::Vector3i(x, y, z));
                if (!map_.isOccupied(index))
                    continue;

                pos = map_.SOGMPtr_->IndexToWorld(index);

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.points.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    inf_pub_.publish(cloud_msg);
}

void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom)
{
    camData_.camera_pos(0) = odom->pose.pose.position.x;
    camData_.camera_pos(1) = odom->pose.pose.position.y;
    camData_.camera_pos(2) = odom->pose.pose.position.z;

    camData_.camera_q = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                           odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    camData_.R_C_2_W = camData_.camera_q.toRotationMatrix() * camData_.R_C_2_B;
    camData_.T_C_2_W = camData_.camera_pos + camData_.T_C_2_B;

    static tf::TransformBroadcaster br;
    Eigen::Quaterniond eq(camData_.R_C_2_W);
    br.sendTransform(tf::StampedTransform(tf::Transform(
                                              tf::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()),
                                              tf::Vector3(camData_.T_C_2_W(0), camData_.T_C_2_W(1), camData_.T_C_2_W(2))),
                                          odom->header.stamp, "map", "base_link"));

    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, camData_.k_depth_scaling_factor);
    }

    cv_ptr->image.copyTo(camData_.depth_image);

    camData_.ptws_hit.clear();
    camData_.ptws_miss.clear();

    Eigen::Vector3d pt_w;
    pcl::PointXYZ pt;
    double depth;

    uint16_t *row_ptr;
    int cols = camData_.depth_image.cols;
    int rows = camData_.depth_image.rows;

    const double inv_factor = 1.0 / camData_.k_depth_scaling_factor;

    if (true)
    {
        local_map_boundary_max_(0) = std::max(local_map_boundary_max_(0), camData_.camera_pos(0));
        local_map_boundary_max_(1) = std::max(local_map_boundary_max_(1), camData_.camera_pos(1));
        local_map_boundary_max_(2) = std::max(local_map_boundary_max_(2), camData_.camera_pos(2));

        local_map_boundary_min_(0) = std::min(local_map_boundary_min_(0), camData_.camera_pos(0));
        local_map_boundary_min_(1) = std::min(local_map_boundary_min_(1), camData_.camera_pos(1));
        local_map_boundary_min_(2) = std::min(local_map_boundary_min_(2), camData_.camera_pos(2));
    }

    for (int v = camData_.depth_filter_margin; v < rows - camData_.depth_filter_margin; v += camData_.skip_pixel)
    {
        row_ptr = camData_.depth_image.ptr<uint16_t>(v) + camData_.depth_filter_margin;

        for (int u = camData_.depth_filter_margin; u < cols - camData_.depth_filter_margin; u += camData_.skip_pixel)
        {
            depth = (*row_ptr) * inv_factor;
            row_ptr = row_ptr + camData_.skip_pixel;

            if (*row_ptr == 0 || depth > camData_.depth_maxdist)
            {
                depth = camData_.depth_maxdist;

                pt_w(0) = (u - camData_.cx) * depth / camData_.fx;
                pt_w(1) = (v - camData_.cy) * depth / camData_.fy;
                pt_w(2) = depth;
                pt_w = camData_.R_C_2_W * pt_w + camData_.T_C_2_W;

                pt.x = pt_w(0);
                pt.y = pt_w(1);
                pt.z = pt_w(2);

                camData_.ptws_miss.points.push_back(pt);
            }
            else if (depth < camData_.depth_mindist)
            {
                continue;
            }
            else
            {
                pt_w(0) = (u - camData_.cx) * depth / camData_.fx;
                pt_w(1) = (v - camData_.cy) * depth / camData_.fy;
                pt_w(2) = depth;
                pt_w = camData_.R_C_2_W * pt_w + camData_.T_C_2_W;

                pt.x = pt_w(0);
                pt.y = pt_w(1);
                pt.z = pt_w(2);

                camData_.ptws_hit.points.push_back(pt);
            }

            if (true)
            {
                local_map_boundary_max_(0) = std::max(local_map_boundary_max_(0), pt_w(0));
                local_map_boundary_max_(1) = std::max(local_map_boundary_max_(1), pt_w(1));
                local_map_boundary_max_(2) = std::max(local_map_boundary_max_(2), pt_w(2));

                local_map_boundary_min_(0) = std::min(local_map_boundary_min_(0), pt_w(0));
                local_map_boundary_min_(1) = std::min(local_map_boundary_min_(1), pt_w(1));
                local_map_boundary_min_(2) = std::min(local_map_boundary_min_(2), pt_w(2));
            }
        }
    }

    depth_need_update_ = true;
}

void updateMapCallback(const ros::TimerEvent &)
{
    static int update_num = 0;
    static double inf_all_t = 0;
    static double inf_max_t = 0;

    if (depth_need_update_ != true)
        return;
    ros::Time t1, t2, t3, t4;
    t1 = ros::Time::now();

    map_.update(&camData_.ptws_hit, &camData_.ptws_miss, camData_.camera_pos);

    t2 = ros::Time::now();

    publishNewOcc();
    publishNewFree();
    publishLocalUpdateRange();
    publishMapRange();

    publishOccMap();
    publishInfMap();

    local_map_boundary_min_ = camData_.camera_pos;
    local_map_boundary_max_ = camData_.camera_pos;

    depth_need_update_ = false;

    t3 = ros::Time::now();

    update_num++;

    inf_all_t = inf_all_t + (t2 - t1).toSec();

    if ((t2 - t1).toSec() > inf_max_t)
        inf_max_t = (t2 - t1).toSec();

    if (update_num % 10 == 0)
    {
        std::cout << "[Inflation] "
                  << "max time: " << inf_max_t << ", average time: " << inf_all_t / update_num << ", time: " << (t2 - t1).toSec() << std::endl;
        std::cout << "[vis] " << (t3 - t2).toSec() << std::endl;
    }
}

void setCameraParam(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["DepthCamera"];
    camData_.depth_heigth = (int)(yaml_node["heigth"]);
    camData_.depth_width = (int)(yaml_node["width"]);
    camData_.fx = (double)(yaml_node["fx"]);
    camData_.fy = (double)(yaml_node["fy"]);
    camData_.cx = (double)(yaml_node["cx"]);
    camData_.cy = (double)(yaml_node["cy"]);

    camData_.k_depth_scaling_factor = (double)(yaml_node["k_depth_scaling_factor"]);
    camData_.depth_maxdist = (double)(yaml_node["depth_maxdist"]);
    camData_.depth_mindist = (double)(yaml_node["depth_mindist"]);
    camData_.depth_filter_margin = (double)(yaml_node["depth_filter_margin"]);
    camData_.skip_pixel = (double)(yaml_node["skip_pixel"]);

    cv::Mat rc2b, tc2b;
    yaml_node["R_C_2_B"] >> rc2b;
    yaml_node["T_C_2_B"] >> tc2b;

    cv::cv2eigen(rc2b, camData_.R_C_2_B);
    cv::cv2eigen(tc2b, camData_.T_C_2_B);

    std::cout << "[CameraParam INIT] use depth camera" << std::endl;
    std::cout << "[CameraParam INIT] depth heigth: " << camData_.depth_heigth << std::endl;
    std::cout << "[CameraParam INIT] depth width: " << camData_.depth_width << std::endl;
    std::cout << "[CameraParam INIT] depth fx: " << camData_.fx << std::endl;
    std::cout << "[CameraParam INIT] depth fy: " << camData_.fy << std::endl;
    std::cout << "[CameraParam INIT] depth cx: " << camData_.cx << std::endl;
    std::cout << "[CameraParam INIT] depth cy: " << camData_.cy << std::endl;
    std::cout << "[CameraParam INIT] depth k_depth_scaling_factor: " << camData_.k_depth_scaling_factor << std::endl;
    std::cout << "[CameraParam INIT] depth depth_maxdist: " << camData_.depth_maxdist << std::endl;
    std::cout << "[CameraParam INIT] depth depth_mindist: " << camData_.depth_mindist << std::endl;
    std::cout << "[CameraParam INIT] depth depth_filter_margin: " << camData_.depth_filter_margin << std::endl;
    std::cout << "[CameraParam INIT] depth skip_pixel: " << camData_.skip_pixel << std::endl;
    std::cout << "[CameraParam INIT] R_C_2_B: \n"
              << camData_.R_C_2_B << std::endl;
    std::cout << "[CameraParam INIT] T_C_2_B: " << camData_.T_C_2_B.transpose() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Inflation_map");
    ros::NodeHandle node("~");

    std::string filename;
    node.param<std::string>("paramfile/path", filename, "./src/gridmap/config/inflation_map.yaml");
    std::cout << "parameter file: " << filename << std::endl;

    map_.init(filename);

    setCameraParam(filename);

    // 订阅和定时器
    map_timer_ = node.createTimer(ros::Duration(0.05), updateMapCallback);

    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node, "/depth", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node, "/odom", 1));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(depthOdomCallback, _1, _2));

    local_update_range_pub_ = node.advertise<visualization_msgs::Marker>("/map/range/local", 10);
    slide_global_map_range_pub_ = node.advertise<visualization_msgs::Marker>("/map/range/slide", 10);
    new_occ_pub_ = node.advertise<sensor_msgs::PointCloud2>("/map/new_occ", 10);
    new_free_pub_ = node.advertise<sensor_msgs::PointCloud2>("/map/new_free", 10);

    occ_pub_ = node.advertise<sensor_msgs::PointCloud2>("/map/sogm", 10);
    inf_pub_ = node.advertise<sensor_msgs::PointCloud2>("/map/Inflation", 10);

    depth_need_update_ = false;

    ros::spin();

    return 0;
}