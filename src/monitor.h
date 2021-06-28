#include "ros/ros.h"
#include <string>
#include <list>
#include <vector>
#include <iterator>
#include <unordered_set>
#include <unordered_map>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "distributed_mapf/RegMsg.h"
#include "distributed_mapf/GoalMsg.h"
#include "distributed_mapf/ClockMsg.h"
#include "visualization/visualization.h"
#include "amrl/vector_map/vector_map.h"

#include "planning/planning.h"
#include "defs.h"
#include "agent.h"

using planning::GraphIndex;
using std::string;

namespace monitor {

class Monitor {
typedef list<GraphIndex>::const_iterator LocItr;
public:
	explicit Monitor(ros::NodeHandle *n): n_(n) ,   collision_num_(0), clock_cnt_(0) {
		viz_pub_ =
    		n_->advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  		viz_msg_ = 
    		visualization::NewVisualizationMessage("map", "monitor");

    	LoadMap();
	};
	
	void InitPublishers() {
    	viz_pub_ = n_->advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  		viz_msg_ = visualization::NewVisualizationMessage("map", "agent");
    }

	void UpdateLocations() {
		//cout << "Updating location of agents. Inc iterators." << endl;
		size_t index = 0;
    	for (auto &loc_it: all_locs_) {
    		if (std::next(loc_it,1) != all_paths_[index].end()) {
    			//cout << "prev loc " << loc_it->pprint(true,true) ;
    			loc_it++;
    			//cout << "incremented loc " << loc_it->pprint(true,true) ;
    		}
    		index++; 
    	}
	}

	void PlanMsgCallback(const distributed_mapf::PathMsg& msg) {
		if (clock_cnt_ == 0) {
			cout << "Clock 0, not started yet." << endl;
			//Not started yet.
			return;
		}
		if (vector_clk_.count(msg.sender_id) > 0 
			&& vector_clk_[msg.sender_id] == msg.agent_vector_clk) {
			return;
		}

		list<GraphIndex> path;
		agent::ConvertPathMsgToGraphIndexList(msg, path);
		AddPath(msg.sender_id, path, msg.clock, msg.agent_vector_clk);

	}

	void ClockMsgCallback(const distributed_mapf::ClockMsg& clkmsg) {
		cout << "Got clock msg " << clkmsg.clock 
			 << "Total collisions: " << collision_num_<< endl;
		UpdateLocations();
		clock_cnt_++;
		if (clock_cnt_ != clkmsg.clock) {
			//sanity check
			cout << "Clokc sync error for monitor." << endl;
			throw;
		}
		//print locations:
		for (int i = 0; i < all_locs_.size(); ++i) {
			all_locs_[i]->pprint(false,false);
			cout << ", ";
		}
		cout << endl;
		CheckCollision();
		Visualize();
	}

	void RegisterMsgCallback(const distributed_mapf::RegMsg& msg) {
		//cout << " Register agent " << msg.sender_id << endl;
		if (agent_colors_.count(msg.sender_id) > 0)
			return;
		agent_colors_[msg.sender_id] = msg.sender_color;
	}


	void AddPath(unsigned int sender_id, list<GraphIndex> path_list,
				 unsigned long agent_clock, unsigned long agent_vector_clk) {

		cout << "Got new path from " << sender_id << ", adding it" << endl;
		//find index from agent_ids array
		if (agent_id_to_index_.count(sender_id) == 0) {
		    //Add path corresponding to agent_id
			all_paths_.push_back(path_list);
			all_locs_.push_back(all_paths_.back().begin());
			cout << "start of path for agent:" << path_list.begin()->pprint(true,true);
			size_t agent_index = all_paths_.size()-1;
			agent_id_to_index_[sender_id] = agent_index;
			vector_clk_[sender_id] = agent_vector_clk;
			alignLocWithClockDiff(agent_clock, agent_index); 
			// sanitiy check
			if (agent_colors_.count(sender_id) == 0) {
				cout << "agent " << sender_id << " did not register properly." << endl;
				throw;
			}

		} else {
			cout << "Updating the plan for agent" << sender_id << endl;
			size_t agent_index = agent_id_to_index_[sender_id];
			all_paths_[agent_index] = path_list;
			all_locs_[agent_index] = all_paths_.back().begin();
			cout << "start of new path for agent:" << path_list.begin()->pprint(true,true);
			// To set the location look at the clock of where that message was
			alignLocWithClockDiff(agent_clock, agent_index);            
		}
	}

	void CheckCollision() {
		for (int i = 0; i < all_locs_.size(); ++i) {
			for (int j = i+1; j < all_locs_.size(); ++j) {
				if (*all_locs_[i] == *all_locs_[j]) {
					cout << "Collision_detected." << endl;
					collision_num_++;
				}
			}
		}
		// TODO: check for collision of agents swapping vertices.
	}

	void Visualize() {
		visualization::ClearVisualizationMsg(viz_msg_);
		VisualizeLoc();
		VisualizePlans();
		viz_pub_.publish(viz_msg_);
	}

	void VisualizeLoc() {
		//cout << "adding loc viz " << endl;
		for (auto &it: agent_colors_) {
			unsigned int agent_id = it.first;
			if (agent_id_to_index_.count(agent_id) == 0)
				continue;
			
			uint32_t agent_color = it.second;
			//cout << "agent id:" << agent_id << endl;
			//cout << "agent color:" << agent_color << endl;
			size_t agent_index = agent_id_to_index_[agent_id];
			GraphIndex agent_vertex = *(all_locs_[agent_index]);
			//agent_vertex.pprint(false,true);
			Eigen::Vector2f agent_loc = graph_.GetLocFromVertexIndex(
													agent_vertex.x, agent_vertex.y); 
			visualization::DrawCross(agent_loc, 0.6, agent_color, viz_msg_);
		}
		//cout << "Done. " << endl;
	}

	void VisualizePlans() {
		for (auto &it: agent_colors_) {
			unsigned int agent_id = it.first;
			if (agent_id_to_index_.count(agent_id) == 0)
				continue;
			uint32_t agent_color = it.second;
			size_t agent_index = agent_id_to_index_[agent_id];
			VisualizePlan(all_paths_[agent_index], agent_color);
		}
	}

	void VisualizePlan(list<planning::GraphIndex> plan, uint32_t color=0x000FF) {
  		for(const auto& node : plan) {
			Eigen::Vector2f node_loc = graph_.GetLocFromVertexIndex(node.x,node.y);
			//std::cout << "[" << node.x << " " << node.y << "] ";
			visualization::DrawCross(node_loc, 0.25, color, viz_msg_);
		}
		//std::cout << std::endl;
		//visualization_pub_.publish(map_viz_msg_);
	}


	/*void Agent::CheckCollisions(unsigned int sender_id) 	{
	//	ROS_INFO("\nDISPLAYING ALL PATHS and size of all_paths is %d\n",all_paths.size());
	//	ROS_INFO("\nSender ID is %d and agent ID is %d\n",sender_id,agent_id_);

		for(unsigned int i = 0;i<all_paths.size();i++){

	//		ROS_INFO("\nPrinting PATH number %d\n",i);
			distributed_mapf::PathMsg msg_i;
			ConvertGraphIndexListToPathMsg(all_paths[i], msg_i);

			int step_i = 0;
			for (const auto& vertex_i: msg_i.path) {
	            unsigned int x_cord = vertex_i.x_id;
                unsigned int y_cord = vertex_i.y_id;		
				int step_j = 0;
	              //          std::cout<<x_cord<<" "<<y_cord<<"\";
				for(unsigned int j = 0; j < all_paths.size(); ++j) {
					if(i != j) {
						distributed_mapf::PathMsg msg_j;
						ConvertGraphIndexListToPathMsg(all_paths[j], msg_j);

						for (const auto& vertex_j: msg_j.path) {
							if(vertex_j.x_id==vertex_i.x_id && vertex_j.y_id==vertex_i.y_id && step_i==step_j){
								ROS_INFO("COLLISION IN PATHS!!!!!\n");
								return;
							}
						}
					}
					step_j++;
				}
				step_i++;
	        }
		}

	ROS_INFO("\nexiting check collision\n");
	}*/

	void LoadMap() {
    	std::string map_file = defs::default_map;
    	std::string full_map_file;
    	if (map_file.find('.') == std::string::npos) {
       		full_map_file = "maps/" + map_file + "/" + map_file + ".vectormap.txt";
    	} else {
       		full_map_file = map_file;
    	}
    	
    	cout << "Loading map...";
    	vector_map::VectorMap map;
    	map.Load(full_map_file);
    	graph_ = planning::Graph(
    	  	 defs::plan_grid_pitch,
		     defs::plan_x_start,
		     defs::plan_x_end,
		     defs::plan_y_start,
		     defs::plan_y_end,
		     defs::plan_num_of_orient,
		     defs::plan_margin_to_wall, map);
    	
    	cout << "Done." << endl;
    }
private: 
	void alignLocWithClockDiff(unsigned long agent_clock, size_t agent_index) {
		unsigned long time_delta = clock_cnt_ - agent_clock;
		if (time_delta < 0) {
            cout << "ERROR: agent_clock is ahead of monitor clock!" << endl;
            throw;
        }
        cout << "--> Timedelta " << time_delta << "incrementing loc " 
        	 << time_delta << "times" << endl;
        for (int i = 0; i < time_delta; i++) {
            if (std::next(all_locs_[agent_index],1) == all_paths_[agent_index].end()) 
            	break;
            all_locs_[agent_index]++;
        }
	}

private:
	ros::NodeHandle* n_;
	planning::Graph graph_;

	vector<list<GraphIndex>> all_paths_;
	vector<LocItr> all_locs_;
	unordered_map<unsigned int, size_t> agent_id_to_index_;
	unordered_map<unsigned int, uint32_t> agent_colors_;

	ros::Publisher viz_pub_;
	amrl_msgs::VisualizationMsg viz_msg_;


	unsigned long clock_cnt_;
	unsigned int collision_num_;
	unordered_map<unsigned int, unsigned long> vector_clk_;

};

};


