#ifndef PLANMAPBASE_H_
#define PLANMAPBASE_H_

#include <iostream>
#include <memory>
#include <limits>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PlanMapBase
{
public:
    PlanMapBase() {}
    virtual ~PlanMapBase() {}

    virtual double getResolution()
    {
        return 0.1;
    }

    /**
     * @brief 获取地图边界
     *
     * @param origin 地图原点，一般是长方体地图的左下角。
     * @param size 地图大小，长方体地图的长宽高。
     */
    virtual void getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size)
    {
        origin = Eigen::Vector3d{-10, -10, -10};
        size = Eigen::Vector3d{20, 20, 20};
    }

    virtual double getOccupancy(const Eigen::Vector3d pos)
    {
        return -1;
    }

    virtual bool isOccupied(const Eigen::Vector3d pos)
    {
        return false;
    }

    virtual bool isOccupied(const Eigen::Vector3i voxel)
    {
        return false;
    }

    virtual bool isOccupied(const Eigen::Vector3d pos, double dist)
    {
        return false;
    }

    virtual bool isOccupied(const Eigen::Vector3i voxel, double dist)
    {
        return false;
    }

    virtual double getDist(const Eigen::Vector3i voxel)
    {
        return std::numeric_limits<double>::infinity();
    }

    virtual double getDist(const Eigen::Vector3d pos)
    {
        return std::numeric_limits<double>::infinity();
    }

    virtual Eigen::Vector3d getGrad(const Eigen::Vector3d pos) {};
    virtual Eigen::Vector3d getCoc(const Eigen::Vector3d pos) {};

    typedef std::shared_ptr<PlanMapBase> Ptr;
};

#endif