
#include "SOGMMap.hpp"

SOGMMap::SOGMMap()
{
}

SOGMMap::~SOGMMap()
{
}

void SOGMMap::init(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["PlanMap"];
    res_ = (double)(yaml_node["resolution"]);
    size_[0] = (double)(yaml_node["sogm_size_x"]);
    size_[1] = (double)(yaml_node["sogm_size_y"]);
    size_[2] = (double)(yaml_node["sogm_size_z"]);

    res_inv_ = 1 / res_;

    for (size_t i = 0; i < 3; i++)
        voxel_num_[i] = size_[i] / res_;

    voxel_num_xy_ = voxel_num_[0] * voxel_num_[1];
    voxel_num_x_ = voxel_num_[0];
    int num = voxel_num_[0] * voxel_num_[1] * voxel_num_[2];

    if (num >= INT_MAX * 0.75)
    {
        std::cout << "[ERROR] init map: the map size is too big!" << std::endl;
        // std::exit(0);
    }

    camera_pos_ = Eigen::Vector3d{0, 0, 0};
    origin_voxel_ = computeVoxelOrigin(camera_pos_);

    raycast_num_ = 0;
    occ_value_ = std::vector<double>(num, SOGMMap::UNKNOWN);

    count_hit_and_miss_ = std::vector<short>(num, 0);
    count_hit_ = std::vector<short>(num, 0);
    flag_traverse_ = std::vector<char>(num, 0);
    flag_rayend_ = std::vector<char>(num, 0);

    double p_hit = (double)(yaml_node["p_hit"]);
    double p_miss = (double)(yaml_node["p_miss"]);
    double p_min = (double)(yaml_node["p_min"]);
    double p_max = (double)(yaml_node["p_max"]);
    double p_occ = (double)(yaml_node["p_occ"]);

    prob_hit_log_ = logit(p_hit);
    prob_miss_log_ = logit(p_miss);
    clamp_min_log_ = logit(p_min);
    clamp_max_log_ = logit(p_max);
    min_occupancy_log_ = logit(p_occ);
    std::cout << "[SOGMMap INIT] map res: " << res_ << " (m)" << std::endl;
    std::cout << "[SOGMMap INIT] map size: " << size_(0) << " " << size_(1) << " " << size_(2) << " (m)" << std::endl;
    std::cout << "[SOGMMap INIT] map origin: " << origin_voxel_(0) << " " << origin_voxel_(1) << " " << origin_voxel_(2) << " (res)" << std::endl;
    std::cout << "[SOGMMap INIT] hit & miss log: " << prob_hit_log_ << " " << prob_miss_log_ << std::endl;
    std::cout << "[SOGMMap INIT] min & max log: " << clamp_min_log_ << " " << clamp_max_log_ << std::endl;
    std::cout << "[SOGMMap INIT] thresh log: " << min_occupancy_log_ << std::endl;
}

// update occupancy
void SOGMMap::setCacheOccupancy(int index, int occ_value)
{
    if (occ_value != 0 && occ_value != 1) // do not set free or occ
        return;

    count_hit_and_miss_[index] += 1;

    if (count_hit_and_miss_[index] == 1) // only traverse once
        cache_index_.push(index);

    if (occ_value == 1)
        count_hit_[index] += 1;
}

void SOGMMap::raycastProcess(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos)
{
    raycast_num_ += 1;
    int vox_idx;
    Eigen::Vector3i pt_voxel, ray_voxel;

    RayCaster raycaster;

    Eigen::Vector3d ray_end = camera_pos * res_inv_;

    for (size_t i = 0; i < ptws_hit_ptr->size(); i++)
    {
        Eigen::Vector3d pt_w{ptws_hit_ptr->at(i).x, ptws_hit_ptr->at(i).y, ptws_hit_ptr->at(i).z};
        Eigen::Vector3i pt_voxel = WorldToVoxel(pt_w);

        if (!isInMap(pt_voxel))
        {
            pt_w = closetPointInMap(pt_w, camera_pos);
            vox_idx = VoxelToIndex(WorldToVoxel(pt_w));
            setCacheOccupancy(vox_idx, 0);
        }
        else
        {
            vox_idx = VoxelToIndex(pt_voxel);
            setCacheOccupancy(vox_idx, 1);
        }

        if (flag_rayend_[vox_idx] == raycast_num_)
        { // every grid only raycast once!
            continue;
        }
        else
        {
            flag_rayend_[vox_idx] = raycast_num_;
        }

        Eigen::Vector3d ray_start = pt_w * res_inv_;
        raycaster.setInput(ray_start, ray_end);

        while (raycaster.step(ray_voxel))
        {
            vox_idx = VoxelToIndex(ray_voxel);
            setCacheOccupancy(vox_idx, 0); // KEY

            if (flag_traverse_[vox_idx] == raycast_num_)
            {
                break;
            }
            else
            {
                flag_traverse_[vox_idx] = raycast_num_;
            }
        }
    }

    for (size_t i = 0; i < ptws_miss_ptr->size(); i++)
    {
        Eigen::Vector3d pt_w{ptws_miss_ptr->at(i).x, ptws_miss_ptr->at(i).y, ptws_miss_ptr->at(i).z};
        Eigen::Vector3i pt_voxel = WorldToVoxel(pt_w);

        if (!isInMap(pt_voxel))
        {
            pt_w = closetPointInMap(pt_w, camera_pos);
            vox_idx = VoxelToIndex(WorldToVoxel(pt_w));
            setCacheOccupancy(vox_idx, 0);
        }
        else
        {
            vox_idx = VoxelToIndex(pt_voxel);
            setCacheOccupancy(vox_idx, 0);
        }

        if (flag_rayend_[vox_idx] == raycast_num_)
        { // every grid only raycast once!
            continue;
        }
        else
        {
            flag_rayend_[vox_idx] = raycast_num_;
        }

        Eigen::Vector3d ray_start = pt_w * res_inv_;
        raycaster.setInput(ray_start, ray_end);

        while (raycaster.step(ray_voxel))
        {
            vox_idx = VoxelToIndex(ray_voxel);
            setCacheOccupancy(vox_idx, 0); // KEY

            if (flag_traverse_[vox_idx] == raycast_num_)
            {
                break;
            }
            else
            {
                flag_traverse_[vox_idx] = raycast_num_;
            }
        }
    }
}

void SOGMMap::beyesProcess()
{
    bool isLastOcc = false;
    bool isNowOcc = false;

    new_occ_.clear();
    new_free_.clear();

    while (!cache_index_.empty())
    {
        int idx_ctns = cache_index_.front();

        if (occ_value_[idx_ctns] == SOGMMap::UNKNOWN)
            occ_value_[idx_ctns] = SOGMMap::DISCOVER;

        if (isOccupied(idx_ctns))
            isLastOcc = true;
        else
            isLastOcc = false;

        double log_odds_update = count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - count_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
        log_odds_update = occ_value_[idx_ctns] + log_odds_update;
        occ_value_[idx_ctns] = std::min(std::max(log_odds_update, clamp_min_log_), clamp_max_log_);

        if (isOccupied(idx_ctns))
            isNowOcc = true;
        else
            isNowOcc = false;

        if (isLastOcc == false && isNowOcc == true)
        {
            new_occ_.push_back(idx_ctns);
        }
        if (isLastOcc == true && isNowOcc == false)
        {
            new_free_.push_back(idx_ctns);
        }

        count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;
        cache_index_.pop();
    }
}

void SOGMMap::update(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr, pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr, Eigen::Vector3d camera_pos = Eigen::Vector3d(0, 0, 0))
{
    slideMap(camera_pos);
    raycastProcess(ptws_hit_ptr, ptws_miss_ptr, camera_pos);
    beyesProcess();
}

// slide map
void SOGMMap::slideMap(const Eigen::Vector3d camera_pos)
{
    slideClearIndex_.clear();

    Eigen::Vector3i new_origin_voxel;
    Eigen::Vector3i voxel_shift;

    camera_pos_ = camera_pos;
    new_origin_voxel = computeVoxelOrigin(camera_pos);

    voxel_shift = new_origin_voxel - origin_voxel_;
    if (voxel_shift(0) == 0 && voxel_shift(1) == 0 && voxel_shift(2) == 0)
    {
        return;
    }

    postShiftIndexes(origin_voxel_, new_origin_voxel);

    for (size_t i = 0; i < slideClearIndex_.size(); i++)
    {
        occ_value_[slideClearIndex_[i]] = SOGMMap::UNKNOWN;
    }

    origin_voxel_ = new_origin_voxel;
}

Eigen::Vector3i SOGMMap::computeVoxelOrigin(const Eigen::Vector3d &camera_pos)
{
    Eigen::Vector3i new_origin_voxel;
    new_origin_voxel = WorldToVoxel(camera_pos);
    for (size_t i = 0; i < 3; i++)
    {
        new_origin_voxel[i] = new_origin_voxel[i] - (voxel_num_[i] >> 1);
    }
    return new_origin_voxel;
}

void SOGMMap::postShiftIndexes(const Eigen::Vector3i origin_voxel, const Eigen::Vector3i new_origin_voxel)
{
    Eigen::Vector3i clear_width;
    Eigen::Vector3i voxel_shift;
    voxel_shift = new_origin_voxel - origin_voxel;

    for (size_t i = 0; i < 3; i++)
    {
        clear_width[i] = std::min(abs(voxel_shift[i]), voxel_num_[i]);
    }

    for (size_t i = 0; i < 3; i++)
    {
        if (voxel_shift[i] > 0)
        {
            getAndClearSlice(0, clear_width[i], i);
        }
        else if (voxel_shift[i] < 0)
        {
            getAndClearSlice(voxel_num_[i] - clear_width[i], clear_width[i], i);
        }
    }
}

void SOGMMap::getAndClearSlice(const int i, const int width, const int dimension)
{
    // set minimum dimensions
    int ixyz_min[3] = {0, 0, 0};
    ixyz_min[dimension] = i;

    // set max dimensions
    int ixyz_max[3] = {voxel_num_(0), voxel_num_(1), voxel_num_(2)};
    ixyz_max[dimension] = i + width;

    Eigen::Vector3i ixyz;
    int ind;
    for (int ix = ixyz_min[0]; ix < ixyz_max[0]; ix++)
    {
        for (int iy = ixyz_min[1]; iy < ixyz_max[1]; iy++)
        {
            for (int iz = ixyz_min[2]; iz < ixyz_max[2]; iz++)
            {
                ixyz(0) = ix + origin_voxel_(0);
                ixyz(1) = iy + origin_voxel_(1);
                ixyz(2) = iz + origin_voxel_(2);

                ind = VoxelToIndex(ixyz);
                slideClearIndex_.push_back(ind);
            }
        }
    }
}

// main interface
std::vector<int> *SOGMMap::getSlideClearIndex()
{
    return &slideClearIndex_;
}

std::vector<int> *SOGMMap::getNewOcc()
{
    return &new_occ_;
}

std::vector<int> *SOGMMap::getNewFree()
{
    return &new_free_;
}

double SOGMMap::getOccupancy(const Eigen::Vector3d pos)
{
    int index = WorldToIndex(pos);
    return occ_value_[index];
}

bool SOGMMap::isOccupied(const Eigen::Vector3i voxel)
{
    int index = VoxelToIndex(voxel);
    return isOccupied(index);
}

bool SOGMMap::isOccupied(const Eigen::Vector3d pos)
{
    int index = WorldToIndex(pos);
    return isOccupied(index);
}

bool SOGMMap::isOccupied(const int index)
{

    if (occ_value_[index] >= min_occupancy_log_)
        return true;
    else
        return false;
}

// get map parameter functions
double SOGMMap::getResolution()
{
    return res_;
}

double SOGMMap::getResInv()
{
    return res_inv_;
}

Eigen::Vector3d SOGMMap::getSize()
{
    return size_;
}

Eigen::Vector3i SOGMMap::getOrigin()
{
    return origin_voxel_;
}

int SOGMMap::getNum()
{
    return voxel_num_[0] * voxel_num_[1] * voxel_num_[2];
}

Eigen::Vector3i SOGMMap::getNum3dim()
{
    return voxel_num_;
}

void SOGMMap::getBoundary(Eigen::Vector3d &origin, Eigen::Vector3d &size)
{
    origin = VoxelToWorld(origin_voxel_);
    size = getSize();
}

// map coordinate convert functions
Eigen::Vector3i SOGMMap::WorldToVoxel(const Eigen::Vector3d pos)
{
    Eigen::Vector3i voxel;
    for (size_t i = 0; i < 3; i++)
    {
        double float_voxels = floor(pos[i] * res_inv_ + 1e-6);
        voxel[i] = static_cast<int>(float_voxels);
        // voxel[i] = floor(pos[i] * res_inv_);
    }
    return voxel;
}

int SOGMMap::VoxelToIndex(const Eigen::Vector3i voxel)
{
    Eigen::Vector3i ixyz_offset;
    for (size_t i = 0; i < 3; i++)
    {
        ixyz_offset[i] = voxel[i] % voxel_num_[i];
        if (ixyz_offset[i] < 0)
            ixyz_offset[i] += voxel_num_[i];
    }

    int index = ixyz_offset[0] + ixyz_offset[1] * voxel_num_x_ + ixyz_offset[2] * voxel_num_xy_;
    return index;
}

Eigen::Vector3i SOGMMap::IndexToVoxel(const int index)
{
    int temp = index;
    Eigen::Vector3i grid, voxel;

    grid[2] = index / voxel_num_xy_;
    temp -= grid[2] * voxel_num_xy_;
    grid[1] = temp / voxel_num_x_;
    temp -= grid[1] * voxel_num_x_;
    grid[0] = temp;

    for (size_t i = 0; i < 3; i++)
    {
        grid[i] = (grid[i] - origin_voxel_[i]) % voxel_num_[i];
        if (grid[i] < 0)
            grid[i] += voxel_num_[i];
    }
    voxel = grid + origin_voxel_;

    return voxel;
}

Eigen::Vector3d SOGMMap::VoxelToWorld(const Eigen::Vector3i voxel)
{
    Eigen::Vector3d pos;
    for (size_t i = 0; i < 3; i++)
    {
        pos(i) = voxel(i) * res_;
        // pos(i) = (voxel(i) + 0.5) * res_; // some problem
    }
    return pos;
}

int SOGMMap::WorldToIndex(const Eigen::Vector3d pos)
{
    return VoxelToIndex(WorldToVoxel(pos));
}

Eigen::Vector3d SOGMMap::IndexToWorld(const int index)
{
    return VoxelToWorld(IndexToVoxel(index));
}

// help functions
Eigen::Vector3d SOGMMap::closetPointInMap(Eigen::Vector3d pos, Eigen::Vector3d camera_pos)
{
    Eigen::Vector3d diff = pos - camera_pos;

    Eigen::Vector3d map_max_boundary;
    Eigen::Vector3d map_min_boundary;
    map_min_boundary = VoxelToWorld(origin_voxel_);
    map_max_boundary = VoxelToWorld(origin_voxel_ + voxel_num_);

    Eigen::Vector3d max_tc = map_max_boundary - camera_pos;
    Eigen::Vector3d min_tc = map_min_boundary - camera_pos;
    double min_t = 1000000;

    for (size_t i = 0; i < 3; ++i)
    {
        if (fabs(diff[i]) > 0)
        {

            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;

            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }

    return camera_pos + (min_t - 0.05) * diff;
}

bool SOGMMap::isInMap(const Eigen::Vector3d pos)
{
    return isInMap(WorldToVoxel(pos));
}

bool SOGMMap::isInMap(const Eigen::Vector3i voxel)
{
    Eigen::Vector3i grid = voxel - origin_voxel_;
    for (size_t i = 0; i < 3; i++)
    {
        if (grid[i] <= 0 || grid[i] >= voxel_num_[i])
        {
            return false;
        }
    }
    return true;
}

void SOGMMap::setObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_hit_ptr)
{
    bool isLastOcc = false;
    bool isNowOcc = false;
    new_occ_.clear();

    for (size_t i = 0; i < ptws_hit_ptr->size(); i++)
    {
        Eigen::Vector3d pt_w{ptws_hit_ptr->at(i).x, ptws_hit_ptr->at(i).y, ptws_hit_ptr->at(i).z};
        Eigen::Vector3i pt_voxel = WorldToVoxel(pt_w);

        if (!isInMap(pt_voxel))
        {
            // std::cout << pt_w.transpose() << " not in the map!" << std::endl;
        }
        else
        {
            int vox_idx = VoxelToIndex(pt_voxel);

            if (isOccupied(vox_idx))
                isLastOcc = true;
            else
                isLastOcc = false;

            occ_value_[vox_idx] = clamp_max_log_; // set occupied

            if (isOccupied(vox_idx))
                isNowOcc = true;
            else
                isNowOcc = false;

            if (isLastOcc == false && isNowOcc == true)
            {
                new_occ_.push_back(vox_idx);
            }
        }
    }
}

void SOGMMap::clearObstacles(pcl::PointCloud<pcl::PointXYZ> *ptws_miss_ptr)
{
    bool isLastOcc = false;
    bool isNowOcc = false;
    new_free_.clear();

    for (size_t i = 0; i < ptws_miss_ptr->size(); i++)
    {
        Eigen::Vector3d pt_w{ptws_miss_ptr->at(i).x, ptws_miss_ptr->at(i).y, ptws_miss_ptr->at(i).z};
        Eigen::Vector3i pt_voxel = WorldToVoxel(pt_w);

        if (!isInMap(pt_voxel))
        {
            // std::cout << pt_w.transpose() << " not in the map!" << std::endl;
        }
        else
        {
            int vox_idx = VoxelToIndex(pt_voxel);

            if (isOccupied(vox_idx))
                isLastOcc = true;
            else
                isLastOcc = false;

            occ_value_[vox_idx] = clamp_min_log_; // set free

            if (isOccupied(vox_idx))
                isNowOcc = true;
            else
                isNowOcc = false;

            if (isLastOcc == true && isNowOcc == false)
            {
                new_free_.push_back(vox_idx);
            }
        }
    }
}

void SOGMMap::clearMap()
{
    int num = voxel_num_[0] * voxel_num_[1] * voxel_num_[2];

    occ_value_ = std::vector<double>(num, SOGMMap::UNKNOWN);

    count_hit_and_miss_ = std::vector<short>(num, 0);
    count_hit_ = std::vector<short>(num, 0);
    flag_traverse_ = std::vector<char>(num, 0);
    flag_rayend_ = std::vector<char>(num, 0);
}
