#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <queue>
#include <vector>
#include <queue>
#include <limits>


namespace planning
{
    class Dijkstra
    {
    private:
        int dir[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
        double goal_x,goal_y,start_x,start_y;
        nav_msgs::OccupancyGrid map;
        int map_height,map_width;
        int index_start,index_goal;
        int origin_x,origin_y;
    public:
        Dijkstra();
        std::vector<int> result;
        nav_msgs::Path path_res;
        nav_msgs::Path get_path();
        void set_params(nav_msgs::OccupancyGrid& Map, geometry_msgs::PoseStamped& Goal, geometry_msgs::PoseStamped& Start);
        void find_path();
        int get_index(int x, int y);
        ~Dijkstra();
    };
    
} // namespace planning


#endif