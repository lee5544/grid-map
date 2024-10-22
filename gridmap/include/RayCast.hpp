//
//  RayCast.hpp
//  uav
//
//  Created by 李勇 on 2023/3/8.
//

#ifndef RayCast_hpp
#define RayCast_hpp

#include <stdio.h>
#include <Eigen/Eigen>
#include <vector>

class RayCaster
{
private:
    /* data */
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    int x_;
    int y_;
    int z_;
    int endX_;
    int endY_;
    int endZ_;
    double maxDist_;
    double dx_;
    double dy_;
    double dz_;
    int stepX_;
    int stepY_;
    int stepZ_;
    double tMaxX_;
    double tMaxY_;
    double tMaxZ_;
    double tDeltaX_;
    double tDeltaY_;
    double tDeltaZ_;
    double dist_;

    int step_num_;

public:
    RayCaster(/* args */)
    {
    }
    ~RayCaster()
    {
    }

    int signum(int x);

    double mod(double value, double modulus);

    double intbound(double s, double ds);

    bool setInput(const Eigen::Vector3d &start,
                  const Eigen::Vector3d &end /* , const Eigen::Vector3d& min,
                  const Eigen::Vector3d& max */
    );
    bool setInput(const Eigen::Vector3d &start,
                  const Eigen::Vector3d &end, const double &min);

    bool step(Eigen::Vector3d &ray_pt);
    bool step(Eigen::Vector3i &ray_pt);
};

#endif /* RayCast_hpp */
