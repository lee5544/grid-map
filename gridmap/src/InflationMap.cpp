//
//  InflationMap.cpp
//  maps
//
//  Created by 李勇 on 2023/3/10.
//

#include "InflationMap.hpp"

void InflationMap::init(std::string filename)
{
    SOGMPtr_.reset(new SOGMMap());
    SOGMPtr_->init(filename);

    int num = SOGMPtr_->getNum();
    inf_value_ = std::vector<int>(num, InflationMap::FREE);

    Eigen::Vector3i x1(-1, 0, 0);
    Eigen::Vector3i x2(1, 0, 0);
    Eigen::Vector3i x3(0, -1, 0);
    Eigen::Vector3i x4(0, 1, 0);
    Eigen::Vector3i x5(0, 0, -1);
    Eigen::Vector3i x6(0, 0, 1);
    connect_vec_.push_back(x1);
    connect_vec_.push_back(x2);
    connect_vec_.push_back(x3);
    connect_vec_.push_back(x4);
    connect_vec_.push_back(x5);
    connect_vec_.push_back(x6);

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["PlanMap"];
    inflate_size_xy_ = (int)(yaml_node["inflate_size_xy"]);
    inflate_size_z_ = (int)(yaml_node["inflate_size_z"]);

    std::cout << "[INFLATION INIT] inflation connect: " << connect_vec_.size() << std::endl;
    std::cout << "[INFLATION INIT] inflate size xy: " << inflate_size_xy_ << " (res)" << std::endl;
    std::cout << "[INFLATION INIT] inflate size z: " << inflate_size_z_ << " (res)" << std::endl;
}

void InflationMap::slide()
{
    std::vector<int> *slide = SOGMPtr_->getSlideClearIndex();
    for (size_t i = 0; i < slide->size(); i++)
    {
        inf_value_[slide->at(i)] = InflationMap::FREE;
    }
}

void InflationMap::update(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos)
{
    std::vector<int> *new_occ, *new_free;
    int index, nbr_idx;
    Eigen::Vector3i origin_voxel, cur_voxel, nbr_voxel;
    Eigen::Vector3i diff;

    std::queue<Eigen::Vector3i> temp_queue;
    std::queue<Eigen::Vector3i> wrong_clear_queue;

    SOGMPtr_->update(ptws_hit_ptr, ptws_miss_ptr, camera_pos);

    new_occ = SOGMPtr_->getNewOcc();
    new_free = SOGMPtr_->getNewFree();
    // std::cout << "new_occ: " << new_occ->size() << " (res)" << std::endl;
    // std::cout << "new_free: " << new_free->size() << " (res)" << std::endl;
    slide();

    for (size_t i = 0; i < new_occ->size(); i++)
    {
        index = new_occ->at(i);

        inf_value_[index] = InflationMap::OCCUPIED;
        origin_voxel = SOGMPtr_->IndexToVoxel(index);

        // //立方体更新
        // Eigen::Vector3i min,max;
        // min = origin_voxel - Eigen::Vector3i(inflate_size_xy_,inflate_size_xy_,inflate_size_z_);
        // max = origin_voxel + Eigen::Vector3i(inflate_size_xy_,inflate_size_xy_,inflate_size_z_);

        // for(int i=min(0); i<max(0); i++)
        //     for(int j=min(0); j<max(0); j++)
        //         for(int k=min(0); k<max(0); k++){
        //             if (!OGMPtr_->SGMPtr_->isInMap(nbr_voxel)) // over local map
        //                 continue;
        //             nbr_idx = OGMPtr_->SGMPtr_->VoxelToIndex(Eigen::Vector3i(i,j,k));

        //             inf_value_[index] = InflationMap::OCCUPIED;
        //         }

        temp_queue.push(origin_voxel);

        while (!temp_queue.empty())
        {
            cur_voxel = temp_queue.front();
            temp_queue.pop();

            for (auto delat : connect_vec_)
            {
                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel)) // over local map
                    continue;

                diff = nbr_voxel - origin_voxel;
                if (abs(diff(0)) > inflate_size_xy_ || abs(diff(1)) > inflate_size_xy_ || abs(diff(2)) > inflate_size_z_) // over update range
                    continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                if (inf_value_[nbr_idx] != InflationMap::OCCUPIED) // for expansion boundary
                {
                    inf_value_[nbr_idx] = InflationMap::OCCUPIED;
                    temp_queue.push(nbr_voxel);
                }
            }
        }
    }

    for (size_t i = 0; i < new_free->size(); i++)
    {
        index = new_free->at(i);

        inf_value_[index] = InflationMap::FREE;

        origin_voxel = SOGMPtr_->IndexToVoxel(index);

        // if (abs(md_.camera_pos(2) * mp_.resolution_inv - origin_voxel(2)) > mp_.inflate_range_z)
        //   continue;

        temp_queue.push(origin_voxel);

        while (!temp_queue.empty())
        {
            cur_voxel = temp_queue.front();
            temp_queue.pop();

            for (auto delat : connect_vec_)
            {
                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel))
                    continue;

                diff = nbr_voxel - origin_voxel;
                if (abs(diff(0)) > 2 * inflate_size_xy_ || abs(diff(1)) > 2 * inflate_size_xy_ || abs(diff(2)) > 2 * inflate_size_z_) // over update range
                    continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);
                if (inf_value_[nbr_idx] == InflationMap::FREE)
                    continue;
                else if (SOGMPtr_->isOccupied(nbr_idx))
                {
                    wrong_clear_queue.push(nbr_voxel);
                }
                else
                {
                    inf_value_[nbr_idx] = InflationMap::FREE;
                    temp_queue.push(nbr_voxel);
                }
            }
        }
    }

    while (!wrong_clear_queue.empty())
    {
        origin_voxel = wrong_clear_queue.front();
        wrong_clear_queue.pop();

        index = SOGMPtr_->VoxelToIndex(origin_voxel);

        inf_value_[index] = InflationMap::OCCUPIED;

        temp_queue.push(origin_voxel);

        while (!temp_queue.empty())
        {
            cur_voxel = temp_queue.front();
            temp_queue.pop();

            for (auto delat : connect_vec_)
            {
                nbr_voxel = cur_voxel + delat;

                if (!SOGMPtr_->isInMap(nbr_voxel))
                    continue;

                diff = nbr_voxel - origin_voxel;
                if (abs(diff(0)) > inflate_size_xy_ || abs(diff(1)) > inflate_size_xy_ || abs(diff(2)) > inflate_size_z_) // over update range
                    continue;

                nbr_idx = SOGMPtr_->VoxelToIndex(nbr_voxel);

                if (inf_value_[nbr_idx] != InflationMap::OCCUPIED)
                {
                    inf_value_[nbr_idx] = InflationMap::OCCUPIED;
                    temp_queue.push(nbr_voxel);
                }
            }
        }
    }
}

double InflationMap::getResolution()
{
    return SOGMPtr_->getResolution();
}

void InflationMap::getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size)
{
    SOGMPtr_->getBoundary(origin, size);
}

bool InflationMap::isOccupied(const Eigen::Vector3d pos)
{
    return isOccupied(SOGMPtr_->WorldToIndex(pos));
}

bool InflationMap::isOccupied(const int index)
{
    if (inf_value_[index] == InflationMap::OCCUPIED)
        return true;
    else
        return false;
}

double InflationMap::getOccupancy(const Eigen::Vector3d pos)
{
    return SOGMPtr_->getOccupancy(pos);
}
