/*
 * @Name:
 * @Author:       yong
 * @Date: 2023-03-12 22:12:10
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-10-22 17:35:43
 * @Description:
 * @Subscriber:
 * @Publisher:
 */

#include "GlobalGridMap.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_map");
    ros::NodeHandle node("~");

    GlobalGridMap glmap;
    glmap.init(node);

    ros::spin();
    return 0;
}