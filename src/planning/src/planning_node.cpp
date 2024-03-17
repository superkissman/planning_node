#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "Dijkstra/Dijkstra.h"
#include "geometry_msgs/PointStamped.h"
#include <sstream>

bool flag = false;


class planning_node
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub,goal_sub; 
    ros::Publisher path_pub;
public:
    /* data */
    nav_msgs::OccupancyGrid Map;
    geometry_msgs::PoseStamped Goal;
    planning::Dijkstra Dij;
    nav_msgs::Path res;
    
    planning_node(ros::NodeHandle& nh) : nh_(nh)
    {

        map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &planning_node::doMap, this);
        goal_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &planning_node::doGoal, this);
        path_pub = nh_.advertise<nav_msgs::Path>("/Path",1);      
    }

    void doMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std::cout << "map" << std::endl;
        Map = *msg;
    }

    void doGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
        std::cout << "Goal message" << std::endl;
        Goal = *msg;
        flag = true;
        // int x = (Goal.pose.position.x - Map.info.origin.position.x)/Map.info.resolution;
        // int y = (Goal.pose.position.y - Map.info.origin.position.y)/Map.info.resolution;
        // int index = y * Map.info.width + x;
        // int index_x = index % Map.info.width;
        // int index_y = index / Map.info.width;
        // double dx = index_x*Map.info.resolution + Map.info.origin.position.x;
        // double dy = index_y*Map.info.resolution + Map.info.origin.position.y;

        geometry_msgs::PoseStamped Start;
        Start.pose.position.x = 0;
        Start.pose.position.y = 0;
        // res.poses.push_back(Start);
        // res.header.frame_id = "map";
        Dij.set_params(Map,Goal,Start);
        Dij.find_path();
        res = Dij.get_path();
        path_pub.publish(res);
    }


    // ~planning_node(){}
};










int main(int argc, char  *argv[])
{   

    ros::init(argc,argv,"planning_node");
    ros::NodeHandle nh("~");


    planning_node planning_example(nh);

    // ros::Duration(5).sleep();
    // while (ros::ok())
    // {
    //     if(flag){
    //         geometry_msgs::PoseStamped Start;
    //         Start.pose.position.x = 0;
    //         Start.pose.position.y = 0;
    //         planning_example.Dij.set_params(planning_example.Map,planning_example.Goal,Start);
    //         planning_example.Dij.find_path();
    //         planning_example.res = planning_example.Dij.get_path();
    //     }
    ros::spin();
    // }


    
    // planning_example.path_pub.publish(res);
    
    // ros::AsyncSpinner spinner(2); // Use multi threads
    // spinner.start();
    
    // ros::waitForShutdown();

    // ros::spin();
    // while (ros::ok())
    // {
    //     planning_example.nh_..publish("dadada");
    // }

    return 0;
}