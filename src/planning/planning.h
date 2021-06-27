/*
 * planning.h
 *
 * Multi-Agent graph and Astar planning algortihm implementation.  
 *
 * Created on: Apr 20, 2021
 *      Author: olegzilm
 */
#ifndef SRC_MAPF_PLANNER_
#define SRC_MAPF_PLANNER_

#include <vector>
#include <list>
#include <utility>
#include <queue>
#include <unordered_map>
#include <utils/common.h>
#include <memory>
#include <iostream>
#include <sstream>


#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


namespace vector_map {
    class VectorMap;
};


using std::vector;
using std::list;
using Eigen::Vector2i;
using std::queue;
using std::deque;
using std::unordered_map;
using std::size_t; 
using std::cout;
using std::endl;

namespace planning {

struct PlanParams {
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


struct GraphIndex {
    GraphIndex():x(0),y(0),orient(0) {};

    GraphIndex(int _x, int _y, int _orient, float check_neg=true) {
        if (check_neg && (_x < 0 || _y < 0 || _orient < 0)) {
            cout << x << " " <<  y << " " << orient << endl;
            cout << "ERROR: GraphIndex::GraphIndex -> indexes cannot be negative " << endl;
            throw;
        }
        x = _x;
        y = _y;
        orient = _orient;
    };

    bool operator==(const GraphIndex& rhs) const {
        if (this->x == rhs.x && this->y == rhs.y && this->orient == rhs.orient)
            return true;
        return false;
    };

    bool operator!=(const GraphIndex& rhs) const {
            if (this->x != rhs.x || this->y != rhs.y || this->orient != rhs.orient)
                return true;
            return false;
    };

    bool operator<(const GraphIndex& rhs) const
    {
           if (this->x > rhs.x)
               return false;
           else if (this->x == rhs.x) {
               if (this->y > rhs.y)
                   return false;
               else if (this->y == rhs.y) {
                   if (this->orient >= rhs.orient)
                       return false;
               }
           }
           return true;
    };

    void operator=(const GraphIndex& rhs)
    {
        this->x = rhs.x;
        this->y = rhs.y;
        this->orient = rhs.orient;
           
    };

    std::string pprint(bool str=false, bool endline=false) const {
        std::stringstream val;
        val << "<" << x <<"," << y << ">";
        if (endline) 
            val << endl;
        if (!str) 
            cout << val.str();
            
        return val.str(); 
    }

    int x;
    int y;
    int orient;
};



void static pprintState(const vector<GraphIndex>& jointState, bool endline) {
    std::stringstream val;
    val << "[";
    for (const auto& index: jointState) {
        val << "<" << index.x << "," << index.y << ">, "; 
    }

    val << "]";
    cout << val.str() << endl;
    
    //return val.str();
    if (endline)
        cout << endl;
};

typedef vector<list<GraphIndex>> vec_1d;
typedef vector<vec_1d> vec_2d;
typedef vector<vec_2d> vec_3d;
typedef vec_3d Vertices;


class Graph {
public:
    explicit Graph() {}; // Just for constuctors of including classes..sucks
    explicit Graph(float grid_spacing, int x_start, int x_end, int y_start,
          int y_end, int num_of_orient, float margin_to_wall, const vector_map::VectorMap& map):
          grid_spacing_(grid_spacing), x_start_(x_start),x_end_(x_end),
          y_start_(y_start),y_end_(y_end),num_of_orient_(num_of_orient),
          margin_to_wall_(margin_to_wall), window_set_(false) {
        if (num_of_orient != 1 && num_of_orient != 4 && num_of_orient != 8) {
            cout << "ERROR: Graph::Graph -> In graph constructor, passed number of orientations should be 4, or 8. Others are not supported." << endl;
            throw;
        }
        GenerateGraph(map);
    };

    void GenerateGraph(const vector_map::VectorMap& map);

    std::list<GraphIndex> GetVertexNeighbors(const GraphIndex& index);
    //std::starlist<GraphIndex> GetVertexNeighbors(const navigation::PoseSE2& pose);

    GraphIndex GetClosestVertex(const navigation::PoseSE2& pose);
    Eigen::Vector2f GetLocFromVertexIndex(const GraphIndex& vIndex) {
        return GetLocFromVertexIndex(vIndex.x, vIndex.y);
    };
    Eigen::Vector2f GetLocFromVertexIndex(int index_x, int index_y);
    
    void SetWindow(const GraphIndex& center, float size_x, float size_y);
    bool IsVertexInWindow(const GraphIndex& vertex) const;
    void UnsetWindow() { window_set_ = false; };
    bool HasWindow() const { return window_set_; }; 

    int getNumVerticesX() const {
        return num_vertices_x_;
    }
    int getNumVerticesY() const{
        return num_vertices_y_;
    }

    int getNumOrient() const{
        return num_of_orient_;
    }

    const Vertices& GetVertices() const {
        return vertices_;
    }

    GraphIndex GetWindowCenter() const {
        return window_center_;
    }

    Vector2i GetWindowVertices() const {
        return window_vertices_;
    }

    /*
    float GetGridSpacing() const {
        return grid_spacing_;
    }
    */


private:
    float NormalizeAngle(float angle);
    list<GraphIndex> removeEdgesWithObstacles(const GraphIndex& vertex,
                                 const list<GraphIndex>& neighbors,
                                 const vector_map::VectorMap& map);
    void GetConnectedNeighbors(const GraphIndex& orient,
                                list<GraphIndex>& neighbors);
    bool checkEdgeForObstacles(geometry::line2f& edge,
                               const vector_map::VectorMap& map);
private:

    float grid_spacing_;
    int x_start_;
    int x_end_;
    int y_start_;
    int y_end_;
    int num_of_orient_;
    float margin_to_wall_;

    Vertices vertices_;
    int num_vertices_x_;
    int num_vertices_y_;

    bool window_set_;
    Vector2i window_vertices_;
    GraphIndex window_center_;
};


class MultiAgentGraph {
public:
    struct Node {
        explicit Node(vector<GraphIndex> state): jointState(state) {};
        //vector<navigation::PoseSE2> poses;

        bool operator==(const Node& rhs) const {
            if (this->jointState == rhs.jointState)
                return true;
            return false;
        };

        std::string pprint(bool return_str = true) {
            std::stringstream val;
            val << "[";
            for (const auto& index: jointState) {
                val << "<" << index.x << "," << index.y << ">, "; 
            }

            val << "]";
            if (!return_str) {
                cout << val.str() << endl;
            }

            return val.str();
            //cout << endl;
        }
        vector<GraphIndex> jointState;
        list<std::shared_ptr<Node>> neighbors;
    };

    typedef std::shared_ptr<Node> NodePtr;
    typedef list<NodePtr> Neighbors;
    typedef std::pair<GraphIndex, GraphIndex> StartGoalPair;

    explicit MultiAgentGraph(): root_(nullptr), 
                                numOfAgents_(0), 
                                agentGraph_(NULL) {};

    bool GetVertex(vector<GraphIndex> state, NodePtr& vertex) const {
        long int flatIndex = GetFlatIndexFromJointState(state);
        unordered_map<long int, NodePtr>::const_iterator it = 
                                                    vertices_.find(flatIndex);
        if (it == vertices_.end()) {
            // throw error?
            std::string msg = "MultiAgentGraph::GetVertex:: Requested Vertex is out of bounds.";
            cout << msg << endl;
            
            throw;
            //return false;
        }
        vertex = it->second;
        return true;
    };

    bool GetVertexNeighbors(vector<GraphIndex> state, Neighbors& neighbors) {
        long int flatIndex = GetFlatIndexFromJointState(state);
        if (vertices_.find(flatIndex) == vertices_.end()) {
            // throw error?
            std::string msg = "MultiAgentGraph::GetVertex:: Requested Vertex is out of bounds.";
            cout << msg << endl;
            throw;
            //return false;
        }
        neighbors = vertices_[flatIndex]->neighbors;
        return true;
    };
    
    //navigation::PoseSE2 GetVertexPose(std::size_t index);
    //navigation::PoseSE2 GetVertexPose(const NodePtr& vertex);
    //std::size_t GetClosestVertex(const navigation::PoseSE2& pose);
    //std::size_t GetNumVertices() {return vertices_.size();};
    //const vector<NodePtr>& GetVertices() const {return vertices_;};
    
    //NodePtr GetVertex(std::size_t index);

    void Init() {
        vertices_.clear();
        agentIndexToAgentId_.clear();
        agentGraph_ = nullptr;
        agentStartsGoals_.clear();
        agentOrigPlans_.clear();
        root_ = nullptr;
        jointStartState_.clear();
        jointGoalState_.clear();
        numOfAgents_ = 0;
    };

    vector<GraphIndex>  GetStartState() const {
        return jointStartState_;
    
    };

    vector<GraphIndex> GetGoalState() const {
        return jointGoalState_;
    };

    NodePtr GetStartNode() const {
        return root_;
    };

    NodePtr GetGoalNode() const {
        NodePtr endNode;
        GetVertex(jointGoalState_, endNode);
        return endNode;
    };

    size_t GetGraphSize() const {
        return vertices_.size();
    };   

    long int GetFlatIndexFromJointState(NodePtr node) const {
        return GetFlatIndexFromJointState(node->jointState);
    };

    unsigned int GetAgentIdFromIndex(size_t agentIndex) {
        return agentIndexToAgentId_[agentIndex];
    }; 

    list<GraphIndex> GetAgentOrigPlan(size_t agentIndex) {
        return agentOrigPlans_[agentIndex];
    }

    unsigned int NumOfAgents() const {
        return numOfAgents_;
    };

    void AddAgentGraph(const Graph& agentGraph) {
        agentGraph_ = std::make_shared<Graph>(agentGraph);
    }

    void SetWindow(const GraphIndex& center, float size_x, float size_y) {
        if (!agentGraph_) {
            cout << "ERROR: MultiAgent graph window set before agent graph was added." << " Do AddAgentGraph before SetWindow" << endl;
            throw;
        }
        agentGraph_->SetWindow(center, size_x, size_y);
        window_set_ = true;
    }

    bool HasWindow() const { return window_set_; }; 

    void AddAgentToJointSpace(unsigned int agentId, const list<GraphIndex>& plan) {
        if (!agentGraph_) {
            cout << "No agent graph was added to MAPF graph. Aborting." << endl;
            return;
        }

        agentIndexToAgentId_.push_back(agentId);
        
        GraphIndex startVertex;
        GraphIndex endVertex;
        if (agentGraph_->HasWindow()) {
            if (!GetStartEndInWindow(plan ,startVertex, endVertex)) {
                cout << "Sanitiy check::AddAgentToJointSpace: path and window don't align for agent." << endl;
                throw;
            }
            //cout << "StartInWin:" << startVertex.pprint(true) << " ";
            cout << "Start/Goal loc for agent in window:";
            debug::print_loc(agentGraph_->GetLocFromVertexIndex(startVertex),"", false); 
            //cout << " EndInWin:"  << endVertex.pprint(true) << " ";
            debug::print_loc(agentGraph_->GetLocFromVertexIndex(endVertex), "", true); 
        
        } else {  
            startVertex = plan.front();
            endVertex = plan.back();
        }
        agentStartsGoals_.push_back(std::make_pair(startVertex, endVertex));
        agentOrigPlans_.push_back(plan);
        numOfAgents_ += 1;

    };

    void MergeAgentGraphs() {
        cout << "DBG::Start Merge graphs to joint space.." << endl;
        // Init vertices vector (using the sizes of all stuffies)
        vertices_.clear();


        if (numOfAgents_ == 0) {
            cout << "No agents were added to MAPF graph. Aborting." << endl;
            return;
        }

        if (!agentGraph_) {
            cout << "No agent graph was added to MAPF graph. Aborting." << endl;
            return;
        }
       
        //cout << "DBG::Add jointStartGoal" << endl;
        
        // Create Start State;
        for (size_t i = 0; i < numOfAgents_; ++i) {
            jointStartState_.push_back(agentStartsGoals_[i].first);
            jointGoalState_.push_back(agentStartsGoals_[i].second);

        }
        
        deque<vector<GraphIndex>> auxQueue;
        queue<NodePtr> nodeQueue;
        
        //cout << "DBG::Add root node " << endl;
        
        root_ = AddNode(jointStartState_, nodeQueue);
        cout << "Set root " << root_->pprint() << endl;
        cout << "Set goal "; pprintState(jointGoalState_,false);
        
        // Start BFS until node queue is empty. Expand neigbors for each node,
        // create nodes for the, and add to the queue and to the curr node's
        // neigbors 
        cout << "Start BFS" << endl;
        int prevVertexSize = 0;
        while (!nodeQueue.empty()) {
            NodePtr currNode = nodeQueue.front();
            
            nodeQueue.pop();
            //cout << "BSF Top Node: " << currNode->pprint() << endl;
        
            // Neigbors are a multiplication of the neigbors of all individual
            // agents. We grow them recursively. i.e start with agent A, number // of states is size(A) and state looks like <state(A)>. The for B 
            // num of states is size(A)xsize(B), state looks <state(A),state(B). // Then we add C and get size(A)xsize(B)xsize(C) etc recursively...
            auxQueue.clear();
            AddNeighbors(currNode, 0, auxQueue, nodeQueue);
            //Neighbors neighbortest;
            //GetVertexNeighbors(currNode->jointState, neighbortest);
            //for (const auto &neighbor: neighbortest) {
            //    cout << neighbor->pprint() << ", "; 
            //}
            if (vertices_.size() > (prevVertexSize+50000)) {
                cout << "Graph current size: " << vertices_.size() << endl;
                prevVertexSize += 50000;
            }
            //cout << endl << endl;


        }    
    };

private:
    //std::size_t AddVertexes(const vector<GraphIndex> jointState, 
    //                      queue<vector< GraphIndex>>& stateQueue) {

    NodePtr AddNode(const vector<GraphIndex>& jointState,
                    queue<NodePtr>& nodeQueue) {
        //cout << "AddNode::" << endl;
        long int flatIndex = GetFlatIndexFromJointState(jointState);
        if (vertices_.find(flatIndex) == vertices_.end()) {
            NodePtr newNode(new Node(jointState));
            vertices_[flatIndex] = newNode;
            nodeQueue.push(newNode);
            return newNode;
        }
        
        return vertices_[flatIndex];

    };
    
    void AddNeighbors(NodePtr currNode, size_t agentIndex ,
                     deque<vector<GraphIndex>>& auxQueue, 
                     queue<NodePtr>& nodeQueue) {
        //cout << "AddNeighbors::Level " << agentIndex << endl;
        if (agentIndex == numOfAgents_) {
            //reached last agent. Stop and add neigbors to the node list
            if (auxQueue.front().size() != numOfAgents_) {
                // Sanitiy check
                cout << "Sanity check failed. Error in implemetation of " 
                     <<  "AddNeighbors recursion end condition" << endl;
                throw;
            }
            //cout << "auxQueue size:: " << auxQueue.size() << " .  Add as neighbors. " << endl;
            while (!auxQueue.empty()) {
                vector<GraphIndex> nodeState = auxQueue.front();
                if (nodeState != currNode->jointState) {
                    NodePtr neighbor = AddNode(nodeState, nodeQueue);
                    currNode->neighbors.push_back(neighbor);
                }
                auxQueue.pop_front();               
            }
            return;
        }

        if (agentIndex == 0) {
            auxQueue.push_back(vector<GraphIndex>());
        }
        
        // sanity check
        size_t stateSize = auxQueue.front().size();
        if (stateSize != agentIndex) {
            cout << "Sanitiy check. Error in AddNeighbors auxQueue implementation." << endl;
            throw;
        }
         
        GraphIndex agentVertex = currNode->jointState[agentIndex];
        while(stateSize == agentIndex) {
            list<GraphIndex> neighbors = agentGraph_->GetVertexNeighbors(agentVertex);
            neighbors.push_back(agentVertex);
            for (auto& neighbor: neighbors) {
                vector<GraphIndex> state =  auxQueue.front();
                if (!CheckCollision(state, neighbor)) {
                    state.push_back(neighbor);
                    auxQueue.push_back(state);
                }
            }
            auxQueue.pop_front();
            stateSize = auxQueue.front().size();
        }
    
        AddNeighbors(currNode, agentIndex+1, auxQueue, nodeQueue);

    };
        
    bool CheckCollision(vector<GraphIndex>& state_so_far,
                        const GraphIndex& agent_index) const {
        for (size_t i = 0; i < state_so_far.size(); i++) {
            if(agent_index == state_so_far[i]) 
                return true;
        }
        return false;
    }

    //bool CheckNoCollisionVertex(const Node& node);
    long int GetFlatIndexFromJointState(const vector<GraphIndex>& state) const {
        long int flatIndex = 0;
        int prevSize = 1;
        //cout << "GetFlatIndexFromJointState:: numAgents:" << agentGraphs_.size() << endl;
        for (int i = numOfAgents_-1; i >= 0; i--) {
            //cout << "GetFlatIndexFromJointState:: i: " << i << endl;
            int sizeX = agentGraph_->getNumVerticesX();
            int sizeY = agentGraph_->getNumVerticesY();
            flatIndex += (prevSize)*(state[i].y*sizeX + state[i].x);
            prevSize *= sizeY * sizeX;
        }
        return flatIndex;
    };


    bool GetStartEndInWindow(const list<GraphIndex>& plan, 
                             GraphIndex& startVertex, GraphIndex& endVertex) {
        bool in_window = false;
        bool passed_center = false;
        for (auto it = plan.cbegin(); it != plan.cend(); it++) {
            if (!in_window) {
                if (agentGraph_->IsVertexInWindow(*it)) {
                    startVertex = *it;
                    in_window = true;
                } 
            } else if (!passed_center) {
                if (*it == agentGraph_->GetWindowCenter()) {
                    passed_center = true;
                }
                else if(!agentGraph_->IsVertexInWindow(*it)) {
                    in_window=false;
                }
            } else {
                if(!agentGraph_->IsVertexInWindow(*it)) {
                    endVertex = *(--it);
                    return true;
                }
            }
        }
        return false;
    };

    //Debug 


private:
    unordered_map<long int, NodePtr> vertices_;
    vector<unsigned int> agentIndexToAgentId_;
    std::shared_ptr<Graph> agentGraph_;
    vector<StartGoalPair> agentStartsGoals_;
    vector<list<GraphIndex>> agentOrigPlans_;
    
    NodePtr root_;
    unsigned int numOfAgents_;
    vector<GraphIndex> jointStartState_;
    vector<GraphIndex> jointGoalState_;

    bool window_set_;
    //GraphIndex window_center_;
    //float window_size_x_;
    //float window_size_y_;
};


class A_star{
public:
    typedef std::pair<double, planning::GraphIndex> element;

public:
    A_star():start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)) {};
    A_star(Graph graph):
          graph_(graph), start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)){};

    bool generatePath(const navigation::PoseSE2& start,
                      const navigation::PoseSE2& goal,
                      const bool& heuristic = true);

    std::map<GraphIndex, float> generateDijCost(const navigation::PoseSE2& loc);

    float getLocationCost(const GraphIndex& index){
        return location_cost_;
    };

    std::list<GraphIndex> getPlan() const { return path_; };

    double calcCost(const GraphIndex& current, const GraphIndex& next);

    double calcHeuristic(const GraphIndex& next);

    bool getPurePursuitCarrot(Eigen::Vector2f center,
                                         float radius,
                                         Eigen::Vector2f& interim_goal);

    GraphIndex GetGoalIndex() const { return goal_; };
    GraphIndex GetStartIndex() const { return start_; };


private:
    void findStartAndGoalVertex(const navigation::PoseSE2& start,
                                const navigation::PoseSE2& goal);

private:
    Graph graph_;   
    std::list<GraphIndex> path_;
    std::list<GraphIndex>::const_iterator curr_path_vertex_;
    GraphIndex start_;
    GraphIndex goal_; 
    float location_cost_;

};


class MultiAgentAstar {
public:
    typedef vector<GraphIndex> JointState;
    typedef std::pair<double, MultiAgentGraph::NodePtr> element;
public:
    //A_star():start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)) {};
    explicit MultiAgentAstar() {};
    explicit MultiAgentAstar(MultiAgentGraph graph, double speed, double time_step) {
        Init(graph, speed, time_step);
    };
    
    void Init(MultiAgentGraph graph, double speed,double time_step) {
        graph_ = graph; 
        speed_ = speed;
        time_step_ = time_step;
        path_.clear();
        goal_cost_ = 0;
    };

    bool GeneratePath(bool heuristic = true);
    //std::map<JointState, float> generateDijCost();

    unsigned int NumOfAgents() const {
        return graph_.NumOfAgents();
    }

    MultiAgentGraph GetGraph() const {
        return graph_;
    }

    void GetAgentPath(list<GraphIndex>& agentPath, unsigned int agentIndex) {
        agentPath.clear();
        for (const auto& vertex: path_) {
            agentPath.push_back(vertex[agentIndex]);
        }
    }

    list<GraphIndex> PlugAgentPath(unsigned int agentIndex) {
        list<GraphIndex> agentPath = graph_.GetAgentOrigPlan(agentIndex);
        PlugAgentPath(agentPath, agentIndex);
        return agentPath;
    }

    void PlugAgentPath(list<GraphIndex>& origPlan, 
                                   unsigned int agentIndex) {
        if (!graph_.HasWindow()) {
            origPlan.clear();
            GetAgentPath(origPlan, agentIndex); 
        } else {
            list<GraphIndex> agentPath;
            GetAgentPath(agentPath, agentIndex);
            list<GraphIndex>::iterator window_start, window_end; 
            for (auto it = origPlan.begin(); it != origPlan.end(); ++it) {
                if (*it == agentPath.front()) {
                    window_start = it;
                } 
                if (*it == agentPath.back()) {
                    window_end = ++it;
                    origPlan.erase(window_start, window_end);
                    origPlan.insert(window_end, agentPath.begin(), agentPath.end());
                    return;
                } 
            }
            cout << "MapfAsterPlanner::ERROR: Could not plug window fix into the original path." << endl;
        }
    }  

    bool HasPlanChanged(unsigned int agentIndex, const list<GraphIndex>& newPlan) {
        const list<GraphIndex>& origPlan = graph_.GetAgentOrigPlan(agentIndex);
        auto orig_it = origPlan.cbegin();
        auto new_it = newPlan.cbegin();
        while(true) {
            if (new_it == newPlan.cend() && orig_it == origPlan.cend())
                return false;
            else if (new_it == newPlan.cend() || orig_it == origPlan.cend())
                return true;
            else if (*new_it != *orig_it) 
                return true;
            orig_it++;
            new_it++;
        }
    }  

    unsigned int GetAgentIdFromIndex(size_t agentIndex) {
        return graph_.GetAgentIdFromIndex(agentIndex);
    } 

private:
    //void findStartAndGoalVertex(const navigation::PoseSE2& start,
    //                            const navigation::PoseSE2& goal);
    double calcCost(const JointState& current, const JointState& next);
    double calcHeuristic(const JointState& next);

private:
    MultiAgentGraph graph_;   
    std::list<JointState> path_;
    //std::list<GraphIndex>::const_iterator curr_path_vertex_;
    float goal_cost_;
    double speed_;
    double time_step_;

};

}

#endif /* SRC_MAPF_PLANNER_ */

