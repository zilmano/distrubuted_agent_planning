/*
 * global_planner.cc
 *
 *  Created on: Nov 15, 2020
 *      Author: olegzilm
 */


#include <cmath>
#include <iterator>
#include <utils/common.h>
#include "planning.h"
#include "amrl/vector_map/vector_map.h"
#include "amrl/math/line2d.h"
#include <unordered_set>


using std::cout;
using std::endl;
using std::priority_queue;
using std::map;
using std::vector;
using std::unordered_set;
namespace planning {
    
    /* 
     * Graph Class
    */

    void Graph::GenerateGraph(const vector_map::VectorMap& map) {
        double num_vertices_x, num_vertices_y;
        float frac = modf((x_end_-x_start_)/grid_spacing_, &num_vertices_x);
        if (frac == 0)
            num_vertices_x -= 1;
        frac = modf((y_end_-y_start_)/grid_spacing_, &num_vertices_y);
        if (frac == 0)
            num_vertices_y_ -= 1;

        // Initialize 3D vector. Each member is a list of Indexes of connected nodes.
        num_vertices_x_ = (int) num_vertices_x;
        num_vertices_y_ = (int) num_vertices_y;
        vertices_ = Vertices(
                num_vertices_x_,
                vec_2d(num_vertices_y_,
                vec_1d(num_of_orient_)));

        // Generate Edges between neighbor vertices
        for (int x = 0; x < num_vertices_x_; ++x) {
           for (int y = 0; y < num_vertices_y_; ++y) {
               for (int o = 0; o < num_of_orient_; ++o) {
                   list<GraphIndex> neighbors;
                   GraphIndex curr_vertex(x,y,o);
                   GetConnectedNeighbors(curr_vertex, neighbors);
                   neighbors = removeEdgesWithObstacles(curr_vertex, neighbors, map);
                   vertices_[x][y][o] = neighbors;
               }
           }
       }
    }


    list<GraphIndex> Graph::removeEdgesWithObstacles(const GraphIndex& vertex,
                                                    const list<GraphIndex>& neighbors,
                                                    const vector_map::VectorMap& map){
        list<GraphIndex> result;
        Eigen::Vector2f vertex_loc = GetLocFromVertexIndex(vertex.x,vertex.y);

        for (auto &neighbor : neighbors) {
            if (neighbor.x >= num_vertices_x_ || neighbor.y >= num_vertices_y_
                    || neighbor.x < 0 || neighbor.y < 0)
                continue;
            Eigen::Vector2f neighbor_loc =
                    GetLocFromVertexIndex(neighbor.x,neighbor.y);
            geometry::line2f edge(vertex_loc, neighbor_loc);
            if (!checkEdgeForObstacles(edge, map))
                result.push_back(neighbor);
        }
        return result;
    }

    bool Graph::checkEdgeForObstacles(geometry::line2f& edge,
                                      const vector_map::VectorMap& map) {
        for (auto &line: map.lines) {
            if (line.CloserThan(edge.p0,edge.p1,margin_to_wall_))
                return true;
        }
        return false;
    }

    void Graph::GetConnectedNeighbors(const GraphIndex& index,
                                      list<GraphIndex>& neighbors) {
       if (num_of_orient_ == 1) {
           neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient,false));
           neighbors.push_back(GraphIndex(index.x+1,index.y,index.orient, false));
           neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x,index.y+1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x-1,index.y,index.orient,  false));
           neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x,index.y-1,index.orient, false));
       } else if (num_of_orient_ == 4) {
           switch (index.orient) {
              case 0:
              neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient,false));
              neighbors.push_back(GraphIndex(index.x+1,index.y,index.orient, false));
              neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient, false));
              break;
              case 1:
              neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x,index.y+1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient, false));
              break;
              case 2:
              neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x-1,index.y,index.orient,  false));
              neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient,  false));
              break;
              case 3:
              neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x,index.y-1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient, false));
              break;
            }
        } else if (num_of_orient_ == 8) {
            cout << "GetConnectedNeighbors:: num_of_orient_==8 not implemented";
            throw;
        } else {
            cout << "ERROR: Graph::GetConnectedNeighbors -> number of orientations should be 4, or 8. Others are not supported." << endl;
            throw;
        }
    }

    GraphIndex Graph::GetClosestVertex(const navigation::PoseSE2& pose) {
        double vertex_num_x, vertex_num_y, vertex_num_orient;
        float vertex_frac_x = modf((pose.loc.x()-x_start_)/grid_spacing_, &vertex_num_x);
        float vertex_frac_y = modf((pose.loc.y()-y_start_)/grid_spacing_, &vertex_num_y);

        float angle = NormalizeAngle(pose.angle);
        //cout << vertex_num_x << " " << vertex_num_y << endl;
        if (vertex_num_x > num_vertices_x_ || vertex_num_y > num_vertices_y_) {
            cout << "ERROR: planning::Graph::getClosestVertex -> Provided pose is outside (above) of planning map bounds" << endl;
            throw;
        }

        if ((vertex_frac_x > 0.5 && (int) vertex_num_x != num_vertices_x_)
                || vertex_num_x == 0)
            vertex_num_x += 1;
        if ((vertex_frac_y > 0.5 && (int) vertex_num_y != num_vertices_y_)
                || vertex_num_y == 0)
            vertex_num_y += 1;

        if (vertex_num_x < 1 || vertex_num_y < 1) {
            cout << "planning::Graph::getClosestVertex --> Provided pose is outside (below) of planning map bounds" << endl;
            throw;
        }

        float vertex_frac_orient = modf(angle/(2*M_PI/(num_of_orient_)), &vertex_num_orient);
        if (vertex_frac_orient > 0.5)
            vertex_num_orient += 1;

        if (vertex_num_orient == num_of_orient_ || num_of_orient_ == 1)
            vertex_num_orient = 0;

        return GraphIndex((int)vertex_num_x-1,
                          (int)vertex_num_y-1,
                          (int)vertex_num_orient);
    }

    std::list<GraphIndex> Graph::GetVertexNeighbors(const GraphIndex& index) {
        if (index.x >= num_vertices_x_
                || index.y >= num_vertices_y_
                || index.orient > num_of_orient_
                || index.x < 0
                || index.y < 0
                || index.orient < 0) {
            cout << "ERROR: planning::Graph::GetVertexNeighbors -> Vertex Index is out of bounds in the graph" << endl;
            throw;
        }
        if (!HasWindow()) {
          return vertices_[index.x][index.y][index.orient];
        } else {
          list<GraphIndex> neighbors;
          for (auto &neighbor : vertices_[index.x][index.y][index.orient]) {
            if (IsVertexInWindow(neighbor)) {
                neighbors.push_back(neighbor);
            }
          }
          return neighbors;
        }
    }

    Eigen::Vector2f Graph::GetLocFromVertexIndex(int index_x,
                                                 int index_y) {
        if (index_x >= num_vertices_x_
              || index_y >= num_vertices_y_
              || index_x < 0
              || index_y < 0) {
            cout << "ERROR: planning::Graph::GetLocFromVertexIndex -> Vertex Index is out of bounds in the graph" << endl;
            throw;
        }
        return Eigen::Vector2f((index_x+1)*grid_spacing_+x_start_,(index_y+1)*grid_spacing_+y_start_);
    }

    void Graph::SetWindow(const GraphIndex& center, float size_x, float size_y) {
        
        window_set_ = true;
        window_center_ = center;
        window_vertices_ = Vector2i((int)((size_x/2)/grid_spacing_),
                                        (int)((size_x/2)/grid_spacing_));
    }

    bool Graph::IsVertexInWindow(const GraphIndex& vertex) const {
        if ( vertex.x >= (window_center_.x - window_vertices_.x()) &&
             vertex.x <= (window_center_.x + window_vertices_.x()) &&
             vertex.y >= (window_center_.y - window_vertices_.y()) &&
             vertex.y <= (window_center_.y + window_vertices_.y())) {
            return true;
        }       
        return false;
    }

    float Graph::NormalizeAngle(float angle) {
        double decimal;
        float frac = modf(angle/(2*M_PI), &decimal);
        if (angle >= 0)
          return frac*2*M_PI;
        else
          return 2*M_PI*(1+frac);
    }

    /*
     * Astar class 
     */


    bool A_star::generatePath(const navigation::PoseSE2& start, 
                              const navigation::PoseSE2& goal, const bool& heuristic){

        //cout << "\n\nStarting generatePath..." << endl;
        //       debug::print_loc(start.loc," start loc", false);
        //       debug::print_loc(goal.loc," goal loc", true);

        findStartAndGoalVertex(start, goal);

        std::priority_queue<element, std::vector<element>, std::greater<element>> frontier;
        std::map<GraphIndex, GraphIndex> came_from;
        std::map<GraphIndex, double> cost_so_far;
        std::map<GraphIndex, bool> expanded; 

        frontier.emplace(0,start_);
        came_from[start_] = start_;
        cost_so_far[start_] = 0;


        while(true){
            if (frontier.empty()) {
                cout << "Astar:: Cannot find path from start to goal. They are not connected.";
                return false; 
            }

            GraphIndex current = frontier.top().second;
            frontier.pop();
           //cout << "Start\t X id:" << start_.x << " Start\t Y id:" << start_.y << std::endl;
           // cout << "Goal\t X id:" << goal_.x << " Goal\t Y id:" << goal_.y << std::endl;
           // cout << "Current X id:" << current.x << " Current Y id:" << current.y << std::endl;
           // cout << "Cost_so_far size: " << cost_so_far.size() << std::endl;
            if(current == goal_){
                break;
            }

            if (expanded.find(current) != expanded.end()) {
                continue; 
            }
            
            expanded[current] = true;

            std::list<GraphIndex> neighbors = graph_.GetVertexNeighbors(current);
            for(auto &neighbor : neighbors){
                //cout << "Neighbor: " << "X id:" << neighbor.x << " Y id:" << neighbor.y << std::endl;
                double new_cost = cost_so_far[current] 
                                  + calcCost(current, neighbor);

                if (heuristic)
                    new_cost += calcHeuristic(neighbor);
                //cout << "Neighbor cost:" << new_cost << " Current Cost:" << cost_so_far[current] << std::endl;
                if(cost_so_far.find(neighbor) == cost_so_far.end() 
                     || new_cost < cost_so_far[neighbor]) {
                    cost_so_far[neighbor] = new_cost;
                    frontier.emplace(new_cost, neighbor);
                    came_from[neighbor] = current;
                    //cout << "New cost found" << std::endl;
                }
            }
        }
        location_cost_ = cost_so_far[goal_];
        cout << "A* start Done." << std::endl;

        std::list<GraphIndex> path;
        GraphIndex curr = goal_;
        while(curr != start_){
            path.push_front(came_from[curr]);
            curr = came_from[curr];
        }

        path_ = path;
        curr_path_vertex_ = path_.begin();
        return true;
    }

    std::map<GraphIndex, float> A_star::generateDijCost(const navigation::PoseSE2& loc){

        GraphIndex location = graph_.GetClosestVertex(loc);
        
        std::priority_queue<element, std::vector<element>, std::greater<element>> frontier;
        std::map<GraphIndex, GraphIndex> came_from;
        std::map<GraphIndex, float> cost_so_far;

        frontier.emplace(0,location);
        came_from[location] = location;
        cost_so_far[location] = 0;

        while(true){
            
            GraphIndex current = frontier.top().second;
            frontier.pop();
           //cout << "Start\t X id:" << start_.x << " Start\t Y id:" << start_.y << std::endl;
           // cout << "Goal\t X id:" << goal_.x << " Goal\t Y id:" << goal_.y << std::endl;
           // cout << "Current X id:" << current.x << " Current Y id:" << current.y << std::endl;
           // cout << "Cost_so_far size: " << cost_so_far.size() << std::endl;

            std::list<GraphIndex> neighbors = graph_.GetVertexNeighbors(current);
            for(auto &neighbor : neighbors){
                //cout << "Neighbor: " << "X id:" << neighbor.x << " Y id:" << neighbor.y << std::endl;
                float new_cost = cost_so_far[current] + A_star::calcCost(current, neighbor);
                //cout << "Neighbor cost:" << new_cost << " Current Cost:" << cost_so_far[current] << std::endl;
                if(cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor]){
                    cost_so_far[neighbor] = new_cost;
                    frontier.emplace(new_cost, neighbor);
                    came_from[neighbor] = current;
                    //cout << "New cost found" << std::endl;
                }
            }
        }
        cout << "Dijkstra's Done." << std::endl;
        return cost_so_far;
    }


    double A_star::calcCost(const GraphIndex& current, const GraphIndex& next){
        return std::sqrt(std::pow(next.x - current.x, 2) + std::pow(next.y - current.y, 2)*1.0);
    }

    double A_star::calcHeuristic(const GraphIndex& next){
        return std::sqrt(std::pow(goal_.x - next.x, 2) + std::pow(goal_.y - next.y, 2)*1.0);
    }

    void A_star::findStartAndGoalVertex(const navigation::PoseSE2& start, const navigation::PoseSE2& goal){
        start_ = graph_.GetClosestVertex(start);
        goal_ = graph_.GetClosestVertex(goal);
    }

    bool A_star::getPurePursuitCarrot(Eigen::Vector2f center,
                                      float radius, Eigen::Vector2f& interim_goal) {
        if (path_.empty())
            return false;
        //cout << " Pure Pursuit Start -- " << endl;
        //debug::print_loc(center, "     Car Loc: ");
        bool intersect_found = false;
        for (auto path_it = curr_path_vertex_;
                path_it != std::prev(path_.end()); ++path_it) {
            GraphIndex curr_vertex_id = *path_it;
            GraphIndex next_vertex_id = *(std::next(path_it));

            Eigen::Vector2f curr_vertex_loc = graph_.GetLocFromVertexIndex(
                    curr_vertex_id.x,
                    curr_vertex_id.y);
            Eigen::Vector2f next_vertex_loc = graph_.GetLocFromVertexIndex(
                                next_vertex_id.x,
                                next_vertex_id.y);
            //debug::print_loc(curr_vertex_loc,"    Curr Vertex:");
            //debug::print_loc(next_vertex_loc,"    Next Vertex:");

            geometry::line2f path_line(curr_vertex_loc, next_vertex_loc);
            Eigen::Vector2f intersect_point_1, intersect_point_2;
            unsigned int num_intersections =
                    geometry::line_circle_intersect(path_line,
                                                    center,
                                                    radius,
                                                    intersect_point_1,
                                                    intersect_point_2);
            if (num_intersections > 0) {
                cout << "        Intersect!" << endl;
                if (num_intersections == 1) {
                   interim_goal = intersect_point_1;
                } else {
                   interim_goal = next_vertex_loc;
                }
                curr_path_vertex_ = path_it;
                cout << "Curr path vertex: (" << curr_vertex_loc.x() << ", "
                        << curr_vertex_loc.y() <<")" << endl;
                intersect_found = true;
            }

        }

      return intersect_found;
    }

    /*
     *  MultiAgentAstar class
     */ 

    bool MultiAgentAstar::GeneratePath(bool heuristic){

        //findStartAndGoalVertex(start, goal);
        MultiAgentGraph::NodePtr start = graph_.GetStartNode();
        MultiAgentGraph::NodePtr goal  = graph_.GetGoalNode();

        cout << "Starting generatePath..." << endl;
        cout << "Start/Goal states:" << endl;
        pprintState(start->jointState, false);
        pprintState(goal->jointState, true);
        //cout << endl;

        priority_queue<element, vector<element>, std::greater<element>> frontier;
        unordered_map<long int, MultiAgentGraph::NodePtr> came_from;
        unordered_map<long int, double> cost_so_far;
        unordered_set<long int> expanded; 

        frontier.emplace(0, start);
        //came_from[start_.jointState] = start_.jointState;
        cost_so_far[graph_.GetFlatIndexFromJointState(start)] = 0;
        
        while (true){
            if (frontier.empty()) {
                cout << "Cannot find path from start to goal!!" << endl;
                return false; 
            }

            MultiAgentGraph::NodePtr current = frontier.top().second;
            frontier.pop();
            //costart_.jointState"Start\t X id:" << start_.x << " Start\t Y id:" << start_.y << std::endl;
            // cout << "Goal\t X id:" << goal_.x << " Goal\t Y id:" << goal_.y << std::endl;
            //cout << endl << "---------------------------------------------" << endl;
            //cout << "Current State: "; pprintState(current->jointState, true);
            //cout << "Cost_so_far size: " << cost_so_far.size() << std::endl;
            
            if(*current == *goal){
                break;
            }

            if (expanded.find(graph_.GetFlatIndexFromJointState(current)) 
                                                             != expanded.end()) {
                continue; 
            }
            
            expanded.insert(graph_.GetFlatIndexFromJointState(current));

            for(auto &neighbor : current->neighbors){
                //cout << "Neighbor: "; pprintState(neighbor->jointState, true);
                double new_cost = 
                    cost_so_far[graph_.GetFlatIndexFromJointState(current)] + 
                    calcCost(current->jointState, neighbor->jointState);
                
                if (heuristic)
                    new_cost += calcHeuristic(neighbor->jointState);

                //cout << "Neighbor cost:" << new_cost << " Current Cost:" << cost_so_far[current] << std::endl;
                if(cost_so_far.find(graph_.GetFlatIndexFromJointState(neighbor)) == cost_so_far.end() 
                    || new_cost < cost_so_far[graph_.GetFlatIndexFromJointState(neighbor)]){
                    cost_so_far[graph_.GetFlatIndexFromJointState(neighbor)] = new_cost;
                    came_from[graph_.GetFlatIndexFromJointState(neighbor)] = current;
                    //cout << "New cost found" << std::endl;
                    frontier.emplace(new_cost, neighbor);

                }
            }
        }
        
        goal_cost_ = cost_so_far[graph_.GetFlatIndexFromJointState(goal)];
        cout << "A* start Done." << endl << endl;

        MultiAgentGraph::NodePtr current = goal;
        path_.push_back(goal->jointState);
        while(current != start){
            path_.push_front(came_from[graph_.GetFlatIndexFromJointState(current)]->jointState);
            current = came_from[graph_.GetFlatIndexFromJointState(current)];
        }

        return true;
        
    }

    double MultiAgentAstar::calcCost(const JointState& currentVertex, 
                                     const JointState& nextVertex){
        // Calculate cost in term of time steps.
        unsigned int cost  = 0;
        JointState goal = graph_.GetGoalState();
        for (size_t i = 0; i < graph_.NumOfAgents(); ++i) {
            unsigned int eucldist = std::sqrt(
                std::pow(nextVertex[i].x - currentVertex[i].x, 2) + 
                std::pow(nextVertex[i].y - currentVertex[i].y, 2)
            );

            if (eucldist == 0) {
                if (nextVertex[i] != goal[i]) {
                    cost += 1;
                }
            } else {
                cost += eucldist/speed_;
            
            }
        }
        return cost;
        
    }

    double MultiAgentAstar::calcHeuristic(const JointState& nextVertex) {

        // Calculate euclidian distanance heuristic, in terms of time steps.
        unsigned int cost  = 0;
        JointState goal = graph_.GetGoalState();
        for (size_t i = 0; i < graph_.NumOfAgents(); ++i) {
            cost += std::sqrt(
                std::pow(nextVertex[i].x - goal[i].x, 2) + 
                std::pow(nextVertex[i].y - goal[i].y, 2)
            );
            cost /= speed_;
        }
        
        return cost;
    }
   
}



