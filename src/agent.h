#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <stdlib.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "amrl/vector_map/vector_map.h"
#include "planning/planning.h"

const std::string plan_topic = "plan_topic";
const std::string map_file = "test_map.txt";

namespace agent {

struct Params {
    float plan_grid_pitch;
    int plan_x_start;
    int plan_x_end;
    int plan_y_start;
    int plan_y_end;
    int plan_num_of_orient;
    float plan_margin_to_wall;
    float replan_dist;
    float pure_pursuit_circ_rad;
    //float w_fpl;
    //float w_clr;
    //float w_dist;
    //float obs_min_clearance;
};

class Agent {

public:
	Agent(ros::NodeHandle *n, Params params, unsigned int id): n_(n), params_(params), agent_id_(id) {}

    void InitPublishers() {
    	plan_publish_ = n_->advertise<distributed_mapf::PathMsg>(plan_topic, 1000);
    	test_publish_ = n_->advertise<std_msgs::String>("test", 1000);
    }

    void PublishPlan(const distributed_mapf::PathMsg& msg) {
    	plan_publish_.publish(msg);	
    }

    void PublishTest(const std_msgs::String& msg) {
    	test_publish_.publish(msg);
    }

    void SetAgentId(unsigned int id) {
    	agent_id_ = id;
    }

    void SetParams(const Params &p) {
    	params_ = p;
    }

    
    void LoadMap() {
    	map_.Load(map_file);
    	graph_ = planning::Graph(params_.plan_grid_pitch,
                              params_.plan_x_start,
                              params_.plan_x_end,
                              params_.plan_y_start,
                              params_.plan_y_end,
                              params_.plan_num_of_orient,
                              params_.plan_margin_to_wall,
                              map_);
    	local_Astar_ = planning::A_star(graph_);
    }

    void PlanMsgCallback(const distributed_mapf::PathMsg& msg);

private:
	ros::NodeHandle* n_;
	ros::Publisher plan_publish_;
	ros::Subscriber plan_subscribe_;
	ros::Publisher test_publish_;

	unsigned int agent_id_;
	planning::Graph graph_;
	planning::A_star local_Astar_;
	planning::A_star global_Astar_;
	vector_map::VectorMap map_;
    
    Params params_;
    	
	//vector clock 
};
};