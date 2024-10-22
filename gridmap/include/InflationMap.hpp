//
//  InflationMap.hpp
//  maps
//
//  Created by Li Yong on 2023/3/10.
//
/****Li Y, Wang L, Ren Y, Chen F, Zhu W.
 * FIImap: Fast Incremental Inflate Mapping for Autonomous MAV Navigation.
 * Electronics. 2023; 12(3):534. https://doi.org/10.3390/electronics12030534
 ****/

#ifndef InflationMap_hpp
#define InflationMap_hpp

#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>
#include <memory>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "SOGMMap.hpp"
#include "PlanMapBase.hpp"

class InflationMap : public PlanMapBase
{
public:
    SOGMMap::Ptr SOGMPtr_;

private:
    enum
    {
        OCCUPIED = 1,
        FREE = 0
    };
    std::vector<int> inf_value_; // inflation value

    std::vector<Eigen::Vector3i> connect_vec_;
    int inflate_size_xy_, inflate_size_z_;

    void slide();

public:
    InflationMap(){};
    ~InflationMap(){};

    void init(std::string filename);

    void update(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos);

    double getResolution() override;
    void getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size) override;
    double getOccupancy(const Eigen::Vector3d pos) override;
    bool isOccupied(const Eigen::Vector3d pos) override;

    bool isOccupied(const int index);
};

#endif /* InflationMap_hpp */
