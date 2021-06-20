#include <random>
#include <string>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <ctime>
#include "ros/ros.h"

#include "defs.h"
#include "distributed_mapf/RegMsg.h"
#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/GoalMsg.h"

//using planning;
using std::cout;
using std::endl;

namespace simulator {
	struct Params {
    	int plan_x_start;
    	int plan_x_end;
    	int plan_y_start;
    	int plan_y_end;
    	int goal_change_freq; // avg number of steps to try and change for
    	float goal_change_prob;

    	//float w_fpl;
    	//float w_clr;
    	//float w_dist;
    	//float obs_min_clearance;
	};
	
	class ClientSim {
	public:
		explicit ClientSim(ros::NodeHandle *n, Params params): n_(n) ,params_(params) {

            gen_.seed(time(0));
			
    		x_dist_ = std::uniform_int_distribution<unsigned int>(
    			params_.plan_x_start, 
    			params_.plan_x_end
    		);
    		
    		y_dist_ = std::uniform_int_distribution<unsigned int>(
    			params_.plan_y_start, 
   				params_.plan_y_end
   			);

   			bernoulli_ = std::bernoulli_distribution(params.goal_change_prob);

   		};

		void SetNewGoal() {
			cout << "SetNewGoal func::" << endl;
			if (agent_registry_.size() == 0) 
				return;

			if (!bernoulli_(gen_)) 
				return; 
			
			distributed_mapf::Vertex goal_vertex; 
			distributed_mapf::GoalMsg new_goal;

			goal_vertex.loc_x = x_dist_(gen_);
			goal_vertex.loc_y = y_dist_(gen_); 

			size_t agent_index = agent_rand_gen_(gen_);
			//cout << agent_index << endl;
			new_goal.target_id = agent_registry_[agent_index];
			new_goal.delay = false;
			new_goal.vertex = goal_vertex;

			cout << "Set new goal for agent " << new_goal.target_id << endl;
			goal_pub_.publish(new_goal);

		};

		void DelayOnce(unsigned int agent_id) {} ;
		//void CheckValidGoal(GraphIndex goal);


		void InitPublishers() {
    		goal_pub_ = 
    			n_->advertise<distributed_mapf::GoalMsg>(defs::new_goal_topic, 1000);
    	}

    	void RegisterMsgCallback(const distributed_mapf::RegMsg& msg) {
    		if (agent_set_.find(msg.sender_id) != agent_set_.end())
				return;

			agent_set_.emplace(msg.sender_id);
    		agent_registry_.push_back(msg.sender_id);
    		agent_rand_gen_.param(
    			std::uniform_int_distribution<unsigned int>::param_type(
    				0, agent_registry_.size()-1
    			)
    		);
    		cout << "register agent id " << msg.sender_id << " at index " 
    			 <<  agent_registry_.size()-1 << endl;
    	}

	private:
		// agent registry.
		std::vector<unsigned int> agent_registry_;
		std::unordered_set<unsigned int> agent_set_; 
		
		Params params_;
		//planning::Graph graph_;
		//planning::A_star planner_;

		std::default_random_engine gen_;
		std::uniform_int_distribution<unsigned int> x_dist_;
		std::uniform_int_distribution<unsigned int> y_dist_;
		std::uniform_int_distribution<unsigned int> agent_rand_gen_;
		std::bernoulli_distribution bernoulli_;

		ros::NodeHandle* n_;
		ros::Publisher goal_pub_;
};

};