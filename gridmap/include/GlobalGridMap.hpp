/*
 * @Name:
 * @Author:       yong
 * @Date: 2023-03-12 21:53:01
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-10-22 14:28:42
 * @Description:
 * @Subscriber:
 * @Publisher:
 */

#ifndef GlobalGridMap_hpp
#define GlobalGridMap_hpp

#include <set>
#include <Eigen/Eigen>
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

struct comp
{
    bool operator()(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2) const
    {
        if (pt1.x < pt2.x)
            return true;
        else if (pt1.x == pt2.x)
        {
            if (pt1.y < pt2.y)
                return true;
            else if (pt1.y == pt2.y)
            {
                if (pt1.z < pt2.z)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false;
    }
};

class GlobalGridMap
{
private:
    std::set<pcl::PointXYZ, comp> GlPoints_;

    ros::Publisher glmap_pub_;
    ros::Subscriber new_occ_sub_, new_free_sub_;
    ros::Subscriber save_map_sub_;
    ros::Timer vis_timer_;

    void newOccCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void newFreeCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    void saveMapCallback(const std_msgs::BoolConstPtr &msg);

    void publishGLmap();
    void visCallback(const ros::TimerEvent &e);

public:
    GlobalGridMap() {};
    ~GlobalGridMap() {};

    int getSize();

    void addPoint(pcl::PointXYZ pt);
    void addPoints(pcl::PointCloud<pcl::PointXYZ> *ptsPtr);

    void erasePoint(pcl::PointXYZ pt);
    void erasePoints(pcl::PointCloud<pcl::PointXYZ> *ptsPtr);

    pcl::PointCloud<pcl::PointXYZ> getMapAll();

    bool saveMap();

    void init(ros::NodeHandle &nh);
};

#endif
