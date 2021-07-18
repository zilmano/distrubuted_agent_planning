#include <random>
#include <string>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <ctime>
#include <thread>
#include <limits>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"


#include "defs.h"
#include "distributed_mapf/RegMsg.h"
#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/GoalMsg.h"

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
            
            x_dist_ = std::uniform_real_distribution<float>(
                (params_.plan_x_start+1.0 ), 
                (params_.plan_x_end-1.0)
            );
            cout << params_.plan_x_start+1 << " , " << params_.plan_x_end-1 << endl;
            
            y_dist_ = std::uniform_real_distribution<float>(
                (params_.plan_y_start+1), 
                (params_.plan_y_end-1)
            );

            reached_goal_wait_gen_ = 
                std::uniform_int_distribution<unsigned int>(
                    defs::reached_goal_wait_time_lower,
                    defs::reached_goal_wait_time_upper
                );

            bernoulli_ = std::bernoulli_distribution(params.goal_change_prob);

        };

        void SetNewGoal(unsigned int agent_id=UINT_MAX) {
            cout << "Running SetNewGoal mechanism..." << endl;
            if (agent_registry_.size() == 0)  {
                cout << "No agents in registry" << endl;
                return;
            }

            if (!bernoulli_(gen_)) { 
                cout << "No generating goal this time." << endl;
                return; 
            }
            
            distributed_mapf::Vertex goal_vertex; 
            distributed_mapf::GoalMsg new_goal;

            goal_vertex.loc_x = x_dist_(gen_);
            goal_vertex.loc_y = y_dist_(gen_); 

            if (agent_id==UINT_MAX) {
                size_t agent_index = agent_rand_gen_(gen_);
                new_goal.target_id = agent_registry_[agent_index];
                cout << "agent index:" << agent_index << "agent_id" << agent_id << endl;
            } else {
                new_goal.target_id = agent_id;
            }
            new_goal.delay = false;
            new_goal.vertex = goal_vertex;
            cout << "Set new goal for agent: " << new_goal.target_id << endl;
            cout << "New  goal:" << "(" 
                 <<  goal_vertex.loc_x  << "," << goal_vertex.loc_y 
                 << ")" << endl;
            goal_pub_.publish(new_goal);

        };

        void DelayOnce(unsigned int agent_id) {} ;
        //void CheckValidGoal(GraphIndex goal);


        void InitPublishers() {
            goal_pub_ = 
                n_->advertise<distributed_mapf::GoalMsg>(defs::new_goal_topic, 1000);
        }

        void RegisterMsgCallback(const distributed_mapf::RegMsg& msg) {
            //cout << "from agent  " << msg.sender_id << endl;
            //cout << " Num of agents in registry:" << agent_registry_.size() << endl;
            if (agent_set_.find(msg.sender_id) != agent_set_.end()) {
                if (msg.request_goal) {
                    std::thread pubthread([this](unsigned int agent_id){
                        unsigned int delay = reached_goal_wait_gen_(gen_);
                        sleep(delay);
                        cout << "\n--------------------------------------------" << endl;
                        cout << "Thread:: Setting new goal for agent " << agent_id 
                             << "that reached its goal, with delay " << delay 
                            << "............." << endl
                            << "---------------------------------------------\n\n";
                        SetNewGoal(agent_id);
                    }, msg.sender_id);
                    pubthread.detach();
                }           
            } else {
                agent_set_.emplace(msg.sender_id);
                agent_registry_.push_back(msg.sender_id);
                agent_goal_.push_back(
                    Eigen::Vector2f(
                        msg.sender_goal.loc_x, msg.sender_goal.loc_y
                    )
                );
                agent_rand_gen_.param(
                    std::uniform_int_distribution<unsigned int>::param_type(
                        0, agent_registry_.size()-1
                    )
                );
                cout << "register agent id " << msg.sender_id << " at index " 
                     <<  agent_registry_.size()-1 << endl;
            }
        }

    private:
        // agent registry.
        std::vector<unsigned int> agent_registry_;
        std::unordered_set<unsigned int> agent_set_; 
        std::vector<Eigen::Vector2f> agent_goal_;

        Params params_;
        //planning::Graph graph_;
        //planning::A_star planner_;

        std::default_random_engine gen_;
        std::uniform_real_distribution<float> x_dist_;
        std::uniform_real_distribution<float> y_dist_;
        std::uniform_int_distribution<unsigned int> agent_rand_gen_;
        std::bernoulli_distribution bernoulli_;
        std::uniform_int_distribution<unsigned int> reached_goal_wait_gen_;                
        
        ros::NodeHandle* n_;
        ros::Publisher goal_pub_;
};

};
