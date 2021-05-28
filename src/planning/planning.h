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


namespace vector_map {
    class VectorMap;
};


using std::vector;
using std::list;
//using Eigen::Vector2i;
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

    int x;
    int y;
    int orient;
};

typedef vector<list<GraphIndex>> vec_1d;
typedef vector<vec_1d> vec_2d;
typedef vector<vec_2d> vec_3d;
typedef vec_3d Vertices;
typedef std::pair<double, planning::GraphIndex> element;


class Graph {
public:
    explicit Graph() {}; // Just for constuctors of including classes..sucks
    explicit Graph(float grid_spacing, int x_start, int x_end, int y_start,
          int y_end, int num_of_orient, float margin_to_wall, const vector_map::VectorMap& map):
          grid_spacing_(grid_spacing), x_start_(x_start),x_end_(x_end),
          y_start_(y_start),y_end_(y_end),num_of_orient_(num_of_orient),
          margin_to_wall_(margin_to_wall) {
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
    Eigen::Vector2f GetLocFromVertexIndex(int index_x, int index_y);
    void AddWallToGraph(const geometry::line2f&  line){};

    int getNumVerticesX(){
        return num_vertices_x_;
    }
    int getNumVerticesY(){
        return num_vertices_y_;
    }

    int getNumOrient(){
        return num_of_orient_;
    }

    const Vertices& GetVertices() const {
        return vertices_;
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
};


class MultiAgentGraph {
public:
    struct Node {
        explicit Node(vector<GraphIndex> state): jointState(state) {};
        //vector<navigation::PoseSE2> poses;
        vector<GraphIndex> jointState;
        list<std::shared_ptr<Node>> neighbors;
    };

    typedef std::shared_ptr<Node> NodePtr;
    typedef list<NodePtr> Neighbors;
    typedef std::pair<GraphIndex, GraphIndex> StartGoalPair;

    explicit MultiAgentGraph(): root_(NULL) {};

    bool GetVertexNeighbors(vector<GraphIndex> state, Neighbors& neighbors) {
        long int flatIndex = GetFlatIndexFromAgentIndexes(state);
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
        agentGraphs_.clear();
        agentStartsGoals_.clear();
        root_ = NULL;
        jointStartState_.clear();
        jointGoalState_.clear();

    };

    void AddAgentToJointSpace(const Graph& agentGraph, unsigned int agentId,                      GraphIndex startVertex, 
                              GraphIndex endVertex) {
        agentIndexToAgentId_.push_back(agentId);
        agentGraphs_.push_back(agentGraph);
        agentStartsGoals_.push_back(std::make_pair(startVertex, endVertex));

    };
    
    void MergeAgentGraphs() {
        
        // Init vertices vector (using the sizes of all stuffies)
        vertices_.clear();


        size_t numAgents = agentGraphs_.size();
       
        // Create Start State;
        for (size_t i = 0; i < numAgents; ++i) {
            jointStartState_.push_back(agentStartsGoals_[i].first);
            jointGoalState_.push_back(agentStartsGoals_[i].second);

        }
        
        deque<vector<GraphIndex>> auxQueue;
        queue<NodePtr> nodeQueue;
        
        root_ = AddNode(jointStartState_, nodeQueue);
        // Start BFS until node queue is empty. Expand neigbors for each node,
        // create nodes for the, and add to the queue and to the curr node's
        // neigbors 
        
        while (!nodeQueue.empty()) {
            NodePtr currNode = nodeQueue.front();
            
            nodeQueue.pop();
            // Neigbors are a multiplication of the neigbors of all individual
            // agents. We grow them recursively. i.e start with agent A, number // of states is size(A) and state looks like <state(A)>. The for B 
            // num of states is size(A)xsize(B), state looks <state(A),state(B). // Then we add C and get size(A)xsize(B)xsize(C) etc recursively...
            auxQueue.clear();
            AddNeighbors(currNode, 0, auxQueue, nodeQueue);
        }    
    };

private:
    //std::size_t AddVertexes(const vector<GraphIndex> jointState, 
    //                      queue<vector< GraphIndex>>& stateQueue) {

    NodePtr AddNode(const vector<GraphIndex>& jointState,
                    queue<NodePtr>& nodeQueue) {
        long int flatIndex = GetFlatIndexFromAgentIndexes(jointState);
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
                     queue<NodePtr> nodeQueue) {
        
        if (agentIndex == agentGraphs_.size()) {
            //reached last agent. Stop and add neigbors to the node list
            if (auxQueue.front().size() != agentGraphs_.size()) {
                // Sanitiy check
                cout << "Sanity check failed. Error in implemetation of " 
                     <<  "AddNeighbors recursion end condition" << endl;
                throw;
            }
            while (!auxQueue.empty()) {
                NodePtr neighbor = AddNode(auxQueue.front(), nodeQueue);
                currNode->neighbors.push_back(neighbor);
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
        Graph agentGraph = agentGraphs_[agentIndex]; 
        GraphIndex agentVertex = currNode->jointState[agentIndex];
        while(stateSize == agentIndex) {
            for (auto& neighbor: agentGraph.GetVertexNeighbors(agentVertex)) {
                vector<GraphIndex> state =  auxQueue.front();
                state.push_back(neighbor);
                auxQueue.push_back(state);
            }
            auxQueue.pop_front();
            stateSize = auxQueue.front().size();
        }
    
        AddNeighbors(currNode, agentIndex+1, auxQueue, nodeQueue);

    };
        
    //void AddEdge(std::size_t vertex,  std::size_t neighbor);
    //bool CheckNoCollisionVertex(const Node& node);
    long int GetFlatIndexFromAgentIndexes(const vector<GraphIndex>& state) 
    {
        long int flatIndex = 0;
        int prevSize = 1;
        
        for (size_t i = agentGraphs_.size()-1; i >= 0; --i) {

            int sizeX = agentGraphs_[i].getNumVerticesX();
            int sizeY = agentGraphs_[i].getNumVerticesY();
            flatIndex += (prevSize)*(state[i].y*sizeX + state[i].x);
            prevSize *= sizeY * sizeX;
        }
        return flatIndex;
    };

private:
    unordered_map<long int, NodePtr> vertices_;
    vector<unsigned int> agentIndexToAgentId_;
    vector<Graph> agentGraphs_;
    vector<StartGoalPair> agentStartsGoals_;
    
    NodePtr root_;
    vector<GraphIndex> jointStartState_;
    vector<GraphIndex> jointGoalState_;
};


class A_star{

public:
    A_star():start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)) {};
    A_star(Graph graph):
          graph_(graph), start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)){};

    std::list<GraphIndex> generatePath(const navigation::PoseSE2& start,
                                       const navigation::PoseSE2& goal,
                                       const bool& heuristic = true);

    std::map<GraphIndex, float> generateDijCost(const navigation::PoseSE2& loc);

    float getLocationCost(const GraphIndex& index){
        return location_cost_;
    };

    double calcCost(const GraphIndex& current, const GraphIndex& next);

    double calcHeuristic(const GraphIndex& next);

    bool getPurePursuitCarrot(Eigen::Vector2f center,
                                         float radius,
                                         Eigen::Vector2f& interim_goal);


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


}

class MultiAgentAstar {
typedef vector<GraphIndex> JointState;
public:
    //A_star():start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)) {};
    A_star(Graph graph, speed, time_step):
          graph_(graph), speed_(speed), time_step_(time_step){};

    std::list<JointState> generatePath(bool heuristic = true);

    std::map<JointState, float> generateDijCost();

    double calcCost(const JointState& current, const JointState& next);

    double calcHeuristic(const JointState& next);

    bool getPurePursuitCarrot(Eigen::Vector2f center,
                                         float radius,
                                         Eigen::Vector2f& interim_goal);


private:
    //void findStartAndGoalVertex(const navigation::PoseSE2& start,
    //                            const navigation::PoseSE2& goal);

private:
    MultuAgentGraph graph_;   
    std::list<JointState> path_;
    //std::list<GraphIndex>::const_iterator curr_path_vertex_;
    //float location_cost_;
    float speed;
    float time_step;

};


}

#endif /* SRC_MAPF_PLANNER_ */

