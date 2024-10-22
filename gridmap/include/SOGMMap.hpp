//
//  Sliding Occupancy Grid Map (SOGM)
//
//  Created by Yong Li on 2023/4/9.
//

#ifndef SOGMMap_hpp
#define SOGMMap_hpp

#include <stdio.h>
#include <iostream>
#include <memory>
#include <queue>
#include <chrono>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "RayCast.hpp"
#include "PlanMapBase.hpp"

#define logit(x) (log((x) / (1 - (x))))

class SOGMMap : public PlanMapBase
{
private:
    double res_, res_inv_; // slide map model
    Eigen::Vector3d size_;
    Eigen::Vector3i voxel_num_;
    Eigen::Vector3i origin_voxel_; // lower_and_left, means the origin of the map

    Eigen::Vector3d camera_pos_;

    int voxel_num_xy_, voxel_num_x_;

    std::vector<double> occ_value_; // log-occ value
    enum
    {
        UNKNOWN = -100,
        DISCOVER = 0
    };

    std::vector<int> slideClearIndex_;

    std::vector<int> new_occ_;
    std::vector<int> new_free_;

    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;

    std::vector<short> count_hit_and_miss_;
    std::vector<short> count_hit_;
    std::queue<int> cache_index_;
    std::vector<char> flag_traverse_, flag_rayend_;
    char raycast_num_;

    // slide map
    Eigen::Vector3i computeVoxelOrigin(const Eigen::Vector3d &new_camera_pos);
    void postShiftIndexes(const Eigen::Vector3i origin_voxel, const Eigen::Vector3i new_origin_voxel);
    void getAndClearSlice(const int i, const int width, const int dimension);
    void slideMap(const Eigen::Vector3d camera_pos);

    // help for update occupancy
    void setCacheOccupancy(int index, int occ_value);
    void raycastProcess(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos);
    void beyesProcess();

public:
    SOGMMap();
    ~SOGMMap();

    void init(std::string filename);

    void update(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos);

    // main interface
    std::vector<int> *getSlideClearIndex();
    std::vector<int> *getNewOcc();
    std::vector<int> *getNewFree();
    double getOccupancy(const Eigen::Vector3d pos) override;
    bool isOccupied(const Eigen::Vector3i voxel) override;
    bool isOccupied(const Eigen::Vector3d pos) override;
    bool isOccupied(const int index);

    // get map parameter functions
    double getResolution() override;
    double getResInv();
    Eigen::Vector3d getSize();
    Eigen::Vector3i getOrigin();
    int getNum();
    Eigen::Vector3i getNum3dim();
    void getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size) override;

    // map coordinate convert functions
    Eigen::Vector3i WorldToVoxel(const Eigen::Vector3d pos);
    int WorldToIndex(const Eigen::Vector3d pos);
    Eigen::Vector3d VoxelToWorld(const Eigen::Vector3i voxel);
    int VoxelToIndex(const Eigen::Vector3i voxel);
    Eigen::Vector3i IndexToVoxel(const int index);
    Eigen::Vector3d IndexToWorld(const int index);

    // help functions
    Eigen::Vector3d closetPointInMap(Eigen::Vector3d pos, Eigen::Vector3d camera_pos);
    bool isInMap(const Eigen::Vector3d pos);
    bool isInMap(const Eigen::Vector3i voxel);

    void setObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr);
    void clearObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr);

    void clearMap();

    typedef std::shared_ptr<SOGMMap> Ptr;
};

#endif
