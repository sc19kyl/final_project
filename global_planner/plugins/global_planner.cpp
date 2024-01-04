#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

GlobalPlanner::GlobalPlanner (){

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}


void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        plan.push_back(start);

	double diffx = goal.pose.position.x - start.pose.position.x;
	double diffy = goal.pose.position.y - start.pose.position.y;
	
	for(int i = 0; i < 2; ++i)
	{
		geometry_msgs::PoseStamped mid_point = goal;
		mid_point.pose.position.x = start.pose.position.x + (0.5 * i * diffx);
		mid_point.pose.position.y = start.pose.position.y + (0.5 * i * diffy);
		
		if(i%2){
			mid_point.pose.position.y += 0.5;
		}
		else
			mid_point.pose.position.y += 0.5;
		plan.push_back(mid_point);

	}
        plan.push_back(goal);

        return true;
    }
};
