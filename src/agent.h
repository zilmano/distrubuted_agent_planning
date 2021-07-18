#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <unordered_set>
#include <iterator>
#include <random>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "distributed_mapf/RegMsg.h"
#include "distributed_mapf/GoalMsg.h"
#include "distributed_mapf/ClockMsg.h"
#include "amrl/vector_map/vector_map.h"
#include "planning/planning.h"
#include "defs.h"


namespace agent {

struct Params {
    float plan_grid_pitch;
    int plan_x_start;
    int plan_x_end;
    int plan_y_start;
    int plan_y_end;
    float window_size_x;
    float window_size_y;
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

static void UpdatePlanToCurrentTime(unsigned long sender_clock_stamp,
                                    unsigned long my_clock_stamp, 
                                    list<planning::GraphIndex>& plan) {
    unsigned long time_delta = (my_clock_stamp - sender_clock_stamp);
    
    cout << "  DBG::updating plan to current time. time delta:" << time_delta 
         << "my " << my_clock_stamp << " his " << sender_clock_stamp <<endl;

    if (sender_clock_stamp > my_clock_stamp) {
        cout << "ERROR: Impossible situation where recieved clock is smaller then sent clock." << endl;
        throw;
    }
    if (plan.size() <= time_delta) {
        auto goal = plan.back(); 
        plan.clear();
        plan.push_back(goal);
    } else {
        for (auto i = 0; i < time_delta; i++) {
            plan.pop_front();
        }
    }
}


class Agent {

public:
    Agent(ros::NodeHandle *n, Params params, unsigned int id, uint32_t color=0x00FF00): 
          n_(n), params_(params), agent_id_(id), done_(false), ideal_(false), 
          clock_cnt_(0), color_(color), own_vector_clk_(0), asked_for_new_goal_(false) {

            gen_.seed(time(0));
            msg_drop_bernoulli_ = std::bernoulli_distribution(defs::msg_drop_prob);
            msg_delay_bernoulli_ = std::bernoulli_distribution(defs::msg_delay_prob); 
            // delay message between 2-6
            msg_delay_time_gen_ = std::uniform_int_distribution<unsigned int>(2, 6);
    }

    void InitPublishers() {
        plan_publish_ = n_->advertise<distributed_mapf::PathMsg>(defs::plan_topic, 1000);
        register_publish_ = n_->advertise<distributed_mapf::RegMsg>(defs::register_topic, 1000); 
        test_publish_ = n_->advertise<std_msgs::String>("test", 1000);
    }

    void PublishPlan(const distributed_mapf::PathMsg& msg) {
        plan_publish_.publish(msg); 
    }

    void PublishTest(const std_msgs::String& msg) {
        test_publish_.publish(msg);
    }

    void PublishRegister(bool request_goal=false) {
        distributed_mapf::RegMsg msg;
        msg.sender_id = agent_id_;
        msg.sender_color = color_;
        
        Eigen::Vector2f current_loc = graph_.GetLocFromVertexIndex(
                                                current_loc_->x,
                                                current_loc_->y); 
                
        msg.sender_loc.loc_x = current_loc.x();
        msg.sender_loc.loc_y = current_loc.y();

        Eigen::Vector2f goal_loc = graph_.GetLocFromVertexIndex(
                                                my_plan_.back().x,
                                                my_plan_.back().y); 
        msg.sender_goal.loc_x = goal_loc.x();
        msg.sender_goal.loc_y = goal_loc.y();

        msg.request_goal = request_goal;

        register_publish_.publish(msg);
    }

    void SetAgentId(unsigned int id) {
        agent_id_ = id;
    }

    void SetParams(const Params &p) {
        params_ = p;
    }

    void SetIdeal() {
        ideal_ = true;
    }

    void ClearIssuedCommands() {
        issued_command_to.clear();
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

    planning::GraphIndex GetCollisionVertex() const {
        return collision_vertex_;
    }

    unsigned long GetClockCnt() const {
        return clock_cnt_;
    }

    unsigned long GetOwnVectorClk() const {
        return own_vector_clk_;
    }

    uint32_t GetColor() const {
        return color_;
    }

    void LoadMap(std::string map_file = "") {
        if (map_file.size() == 0) {
            map_file = defs::default_map;
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

    void GoalMsgCallback(const distributed_mapf::GoalMsg& msg);

    void ClockMsgCallback(const distributed_mapf::ClockMsg& clkmsg) {
        auto clock_diff = clkmsg.clock - clock_cnt_;
        if (clock_diff < 0)
            cout << "Impossible situation. Centralized clock is falling behind agent clock.";
        
        if (clock_cnt_ == 0) { 
            cout << endl << "Got First clock. Doing nothing." << endl;
            clock_cnt_ = clkmsg.clock;
        } else {
            cout << endl << "Got clock. clock diff is " << clock_diff << ", increading loc." << endl;
            
            clock_cnt_ = clkmsg.clock;
            for (auto i = 0; 
                i < clock_diff && std::next(current_loc_, 1) != my_plan_.end(); ++i) {
                current_loc_++;
            }
            if (std::next(current_loc_, 1) == my_plan_.end()) {
                cout << "Agent has reached its goal. Idle." << endl;
                if (!asked_for_new_goal_) {
                    cout << "Asking clientsim for new goal..." << endl;
                    PublishRegister(true);
                    asked_for_new_goal_ = true; 
                }
            }
        }
        cout << "Got clock " << clkmsg.clock << ". Current loc:" << current_loc_->pprint(true, true);
        my_plan_.erase(my_plan_.begin(),current_loc_);
    }

    bool Plan(const navigation::PoseSE2& start, 
              const navigation::PoseSE2& goal) {
        cout << "Do agent local planning." << endl;
        done_ = local_Astar_.generatePath(start, goal);
        if (done_) {
            my_plan_ = local_Astar_.getPlan();
            own_vector_clk_++;
            current_loc_ = my_plan_.begin();
            return true;
        } else {
            cout << "ERROR: could not find path from start to goal" << endl;
            return false;
        }
    };


    bool DetectCollision(list<planning::GraphIndex> other_plan) {
        list<planning::GraphIndex>::const_iterator myit, otherit, 
                                                   prev_myit, prev_otherit;
        // Align Plan length
        auto my_plan = my_plan_;
        if (my_plan.size() > other_plan.size()) {
            auto padding = my_plan.size() - other_plan.size();
            other_plan.insert(other_plan.end(), 
                              padding, other_plan.back());
        } else {
            auto padding = other_plan.size() - my_plan.size();
            my_plan.insert(my_plan.end(), 
                           padding, my_plan.back());
        }

        prev_myit = my_plan.end();
        myit      = my_plan.begin();
        otherit   = other_plan.begin();
        
        //cout << "Update plans (mine then other):" << endl;
        //planning::PrintPlan(my_plan);
        //planning::PrintPlan(other_plan);
        cout << "Start detection collision:" << endl;
        do {
            cout << " [" << myit->pprint(true) << ", " << otherit->pprint(true) << "]"; 
            if (*myit == *otherit) { 
                cout << endl;
                collision_vertex_ = *myit;
                return true;
            }

            if (prev_myit != my_plan.end() && 
                *myit == *prev_otherit && *prev_myit == *otherit) {
                collision_vertex_ = *myit;
                cout << endl;
                return true;
            }
            prev_myit = myit; 
            prev_otherit = otherit;
            if (std::next(myit,1) != my_plan.end())
                myit++;
            if (std::next(otherit, 1) != other_plan.end())
                otherit++;
        } while (std::next(myit,1) != my_plan.end() || 
                 next(otherit, 1) != other_plan.end());
        cout << endl << endl;
        return false;
    }

private:

    navigation::PoseSE2 IndexToLoc(planning::GraphIndex index){
        navigation::PoseSE2 loc;
        loc.loc = graph_.GetLocFromVertexIndex(index.x, index.y);
        return loc; 
    }

    void JointReplan(const list<planning::GraphIndex>& recieved_plan, 
                     unsigned int other_agent_id);

    void ChangePlan(const distributed_mapf::PathMsg& msg);

    void networkModel();


    

private:
    ros::NodeHandle* n_;
    ros::Publisher plan_publish_;
    ros::Publisher register_publish_;
    ros::Publisher test_publish_;

    unsigned int agent_id_;
    planning::Graph graph_;
    planning::A_star local_Astar_;
    planning::MultiAgentAstar mapf_Astar_;
    vector_map::VectorMap map_;
    
    Params params_;
    
    bool done_;
    bool ideal_; // ideal communication conditions.
    list<planning::GraphIndex> my_plan_;
    //navigation::PoseSE2 current_loc_;
    list<planning::GraphIndex>::const_iterator current_loc_;
    planning::GraphIndex collision_vertex_;

    //state
    // TODO: Change this state to a better one, once we stamp the command 
    //       message back to the notify that trigered it, that the best way to keep on track.
    //       may kind of like a vector clock.
    std::unordered_set<unsigned int> issued_command_to; 

    // "Physical" clock
    unsigned long clock_cnt_;
    uint32_t color_;

    // vector clock;
    unsigned long own_vector_clk_;
    unordered_map<unsigned int, unsigned long> vector_clk_;

    bool asked_for_new_goal_;

    // network un-reliabilty model
    std::default_random_engine gen_;
    std::bernoulli_distribution msg_drop_bernoulli_;
    std::bernoulli_distribution msg_delay_bernoulli_;
    std::uniform_int_distribution<unsigned int> msg_delay_time_gen_;

};

}
