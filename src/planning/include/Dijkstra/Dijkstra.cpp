#include "Dijkstra/Dijkstra.h"



namespace planning{

    Dijkstra::Dijkstra(){
        ROS_INFO("Dijkstra Has Been Structed!");
    }

    void Dijkstra::set_params(nav_msgs::OccupancyGrid& Map, geometry_msgs::PoseStamped& Goal, geometry_msgs::PoseStamped& Start){
        origin_x = Map.info.origin.position.x;
        origin_y = Map.info.origin.position.y;
        goal_x = (Goal.pose.position.x - Map.info.origin.position.x)/Map.info.resolution;
        goal_y = (Goal.pose.position.y - Map.info.origin.position.y)/Map.info.resolution;
        start_x = (Start.pose.position.x - Map.info.origin.position.x)/Map.info.resolution;
        start_y = (Start.pose.position.y - Map.info.origin.position.y)/Map.info.resolution;
        map = Map;
        map_height = Map.info.height;
        map_width = Map.info.width;
        index_goal = goal_y * map_width + goal_x;
        index_start = start_y * map_width + start_x;
        std::cout << "index_goal:" << index_goal << std::endl;
        std::cout << "index_start:" << index_start << std::endl;
    }

    void Dijkstra::find_path(){
        nav_msgs::Path res;
        
        std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,std::greater<std::pair<int,int>>> q;

        q.push(std::make_pair(0,index_start));
        std::vector<int> dist(map_height*map_width,INT16_MAX);
        std::vector<bool> visted(map_height*map_width,false);
        std::vector<int> path(map_height*map_width);

        dist[index_start] = 0;
        // path[index_start] = index_start;
        int dirs[4] = {+1,-1,map_width,-map_width};
        while(q.top().second != index_goal){

            int index = q.top().second;
            q.pop();

            // if(index == index_goal){
            //     break;
            // }

            if(visted[index]){
                continue;
            }

            visted[index] = true;

            if(map.data[index] != 0){
                continue;
            }

            int x = index % map_width;
            int y = index / map_width;
            

            for(int i = 0;i < 4; ++i){
                // int nx = x + dir[i][0];
                // int ny = y + dir[i][1];
                // if(nx>=0 && nx<map_width && ny>=0 && ny<map_height && (dist[index]+1)<dist[get_index(nx,ny)]){
                //     dist[get_index(nx,ny)] = dist[index] + 1;
                //     q.emplace(dist[index] + 1, get_index(nx,ny));
                //     path[get_index(nx,ny)] = index;
                // }
                int new_index = index + dirs[i];
                if(new_index>=0 && new_index < (map_height*map_width) && (dist[index]+1)<dist[new_index]){
                    dist[new_index] = dist[index] + 1;
                    q.push(std::make_pair(dist[index] + 1, new_index));
                    path[new_index] = index;
                }
            } 
        }

        int re_index = index_goal;
        result.push_back(index_goal);
        while(path[re_index] != index_start){
            result.push_back(path[re_index]);
            re_index = path[re_index];
        }
        
        std::cout << result.size() << std::endl;
        std::cout << dist[index_goal] << std::endl;
        for(int k = 0; k < result.size(); ++k){
            int x = result[k] % map_width;
            int y = result[k] / map_width;
            geometry_msgs::PoseStamped poses;
            poses.pose.position.x = x * map.info.resolution + map.info.origin.position.x;
            poses.pose.position.y = y * map.info.resolution + map.info.origin.position.y;
            res.header.frame_id="map";
            res.poses.push_back(poses);
        }
        path_res = res;

    }

    nav_msgs::Path Dijkstra::get_path(){
        path_res.poses.clear();
        return path_res;
    }

    int Dijkstra::get_index(int x, int y){
        return y * map_width + x;
    }



    Dijkstra::~Dijkstra(){}
    

}
