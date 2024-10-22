/*
 * @Name:
 * @Author:       yong
 * @Date: 2023-03-12 22:03:46
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-09-11 19:58:43
 * @Description:
 * @Subscriber:
 * @Publisher:
 */

#include "GlobalGridMap.hpp"

int GlobalGridMap::getSize()
{
    return GlPoints_.size();
}

void GlobalGridMap::addPoint(pcl::PointXYZ pt)
{
    GlPoints_.insert(pt);
}

void GlobalGridMap::addPoints(pcl::PointCloud<pcl::PointXYZ> *ptsPtr)
{
    for (std::size_t i = 0; i < ptsPtr->size(); i++)
        GlPoints_.insert(ptsPtr->at(i));
}

void GlobalGridMap::erasePoint(pcl::PointXYZ pt)
{
    GlPoints_.erase(pt);
}

void GlobalGridMap::erasePoints(pcl::PointCloud<pcl::PointXYZ> *ptsPtr)
{
    for (std::size_t i = 0; i < ptsPtr->size(); i++)
        GlPoints_.erase(ptsPtr->at(i));
}

pcl::PointCloud<pcl::PointXYZ> GlobalGridMap::getMapAll()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::set<pcl::PointXYZ, comp>::iterator itr;
    for (itr = GlPoints_.begin(); itr != GlPoints_.end(); itr++)
    {
        cloud.push_back(*itr);
    }

    return cloud;
}

bool GlobalGridMap::saveMap()
{
    time_t currentTime = time(NULL);
    char chCurrentTime[256];
    strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d %H%M%S", localtime(&currentTime));
    std::string stCurrentTime = chCurrentTime;
    std::string filename = stCurrentTime + "GridMap" + ".ply";

    pcl::PointCloud<pcl::PointXYZ>
        cloud = getMapAll();
    pcl::io::savePLYFile(filename, cloud);

    return true;
}

void GlobalGridMap::init(ros::NodeHandle &nh)
{
    new_occ_sub_ = nh.subscribe("/map/new_occ", 1, &GlobalGridMap::newOccCallback, this);
    new_free_sub_ = nh.subscribe("/map/new_free", 10, &GlobalGridMap::newFreeCallback, this);
    save_map_sub_ = nh.subscribe("/map/save", 1, &GlobalGridMap::saveMapCallback, this);

    glmap_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/occupancy/global", 10);

    vis_timer_ = nh.createTimer(ros::Duration(0.1), &GlobalGridMap::visCallback, this);

    std::cout << "[GLOBAL MAP] init" << std::endl;
}

void GlobalGridMap::newOccCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    addPoints(&cloud);
}

void GlobalGridMap::newFreeCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    erasePoints(&cloud);
}

void GlobalGridMap::saveMapCallback(const std_msgs::BoolConstPtr &msg)
{
    if (msg->data == true)
    {
        saveMap();
    }
}

void GlobalGridMap::publishGLmap()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud = getMapAll();

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    glmap_pub_.publish(cloud_msg);
}

void GlobalGridMap::visCallback(const ros::TimerEvent &e)
{
    publishGLmap();
}
