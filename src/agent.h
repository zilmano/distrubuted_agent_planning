#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <stdlib.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "amrl/vector_map/vector_map.h"
#include "planning/planning.h"

const std::string plan_topic = "plan_topic";
//const std::string location_topic = "loc_topic";
const std::string default_map = "GDC1";

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

static void ConvertPathMsgToGraphIndexList(const distributed_mapf::PathMsg& msg, 
	                                      list<planning::GraphIndex>& plan) {
	for (const auto& vertex: msg.path) {
		plan.push_back(planning::GraphIndex((unsigned int)vertex.x_id,
											(unsigned int)vertex.y_id,0));
	}
}

static void ConvertGraphIndexListToPathMsg(const list<planning::GraphIndex>& plan,
												 distributed_mapf::PathMsg& msg) {
	for (const auto& graphIndex: plan) {
		distributed_mapf::Vertex vertex;
		vertex.x_id = graphIndex.x;
		vertex.y_id = graphIndex.y;
		msg.path.push_back(vertex);
	}
}


class Agent {

public:
	Agent(ros::NodeHandle *n, Params params, unsigned int id): 
		n_(n), params_(params), agent_id_(id), done_(false) {}

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

    planning::Graph GetLocalGraph() const {
    	return graph_;
    }

	list<planning::GraphIndex> GetPlan() const {
    	return my_plan_;
    }

    planning::GraphIndex GetStartVertex() const {
    	return local_Astar_.GetStartIndex();
    }

	planning::GraphIndex GetGoalVertex() const {
    	return local_Astar_.GetGoalIndex();
    }    

    void LoadMap(std::string map_file = "") {
    	if (map_file.size() == 0) {
    		map_file = default_map;
    	}

    	std::string full_map_file;
    	if (map_file.find('.') == std::string::npos) {
       		full_map_file = "maps/" + map_file + "/" + map_file + ".vectormap.txt";
    	} else {
       		full_map_file = map_file;
    	}
    	
    	map_.Load(full_map_file);
    	graph_ = planning::Graph(params_.plan_grid_pitch,
                              params_.plan_x_start,
                              params_.plan_x_end,
                              params_.plan_y_start,
                              params_.plan_y_end,
                              params_.plan_num_of_orient,
                              params_.plan_margin_to_wall,
                              map_);
    	
    	local_Astar_ = planning::A_star(graph_);
    	//mapf_Astar_.Init(graph_, );


    }

    void PlanMsgCallback(const distributed_mapf::PathMsg& msg);

    void check_collisions(int sender_id);
    void add_path(int sender_id, list<planning::GraphIndex> path_list);

    void Plan(const navigation::PoseSE2& start, 
    		  const navigation::PoseSE2& goal) {

    	done_ = local_Astar_.generatePath(start, goal);
    	if (done_) {
    		my_plan_ = local_Astar_.getPlan();
		} else {
			cout << "ERROR: could not find path from start to goal" << endl;
		}

    };

private:
	
    	bool DetectCollision(const list<planning::GraphIndex>& other_path) {
        list<planning::GraphIndex>::const_iterator myit, otherit;
        auto my_path = local_Astar_.getPlan();
		for (myit = my_path.begin(),otherit = other_path.begin();
			 myit != my_path.end() && otherit != other_path.begin();
			 otherit++, myit++) {

			if (*myit == *otherit) 
				return true;
			
		}
		return false;
	}

	bool DetectCollision_monitor(const list<planning::GraphIndex>& other_path, const list<planning::GraphIndex>& my_path) {
        list<planning::GraphIndex>::const_iterator myit, otherit;
                for (myit = my_path.begin(),otherit = other_path.begin();
                         myit != my_path.end() && otherit != other_path.begin();
                         otherit++, myit++) {

                        if (*myit == *otherit)
                                return true;

                }
                return false;
        }


	void JointReplan(const list<planning::GraphIndex>& recieved_plan, 
					 unsigned int other_agent_id);

	void ChangePlan(const distributed_mapf::PathMsg& msg);

	void networkModel();


    

private:
	ros::NodeHandle* n_;
	ros::Publisher plan_publish_;
	ros::Subscriber plan_subscribe_;
	ros::Publisher test_publish_;

	unsigned int agent_id_;
	planning::Graph graph_;
	planning::A_star local_Astar_;
	planning::MultiAgentAstar mapf_Astar_;
	planning::MultiAgentGraph mapf_graph_;
	vector_map::VectorMap map_;
    
    Params params_;

    bool done_;
    list<planning::GraphIndex> my_plan_;
	//vector clock 
};
}
