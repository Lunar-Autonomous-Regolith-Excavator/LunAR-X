#ifndef TASK_OPTIMIZER_H
#define TASK_OPTIMIZER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/action/plan_task.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lx_msgs/msg/berm_section.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/utils.h>

#include <vector>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#define GETMAPINDEX(x, y, width) (y * width + x)

using namespace std;

// ----------------- Structs -----------------
struct Bounds {
    double x_min;
    double x_max;
    double y_min;
    double y_max;

    Bounds() {
        x_min = std::numeric_limits<double>::max();
        x_max = std::numeric_limits<double>::min();
        y_min = std::numeric_limits<double>::max();
        y_max = std::numeric_limits<double>::min();
    }
};

struct Pose2D {
    double x;
    double y;
    double theta;

    Pose2D() {}

    Pose2D(double x, double y, double theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }
};

struct Point2D {
    int x;
    int y;

    bool operator==(const Point2D& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point2D& other) const {
        return !(*this == other);
    }
};

// define a custom comparator for priority queue
template <class T>
struct compare_pair
{
    bool operator()(pair<int, T> p1, pair<int, T> p2)
    {
        return p1.first > p2.first;
    }
};

// Struct for storing a state
struct TaskState
{
    vector<int> visited_berm_count;
    vector<bool> visited_excavations;
    int current_exc_site; // with respect to the excavation poses
    int num_dumps_per_segment;

    // Default constructor
    TaskState(int D, int E, int num_dumps_per_segment)
    {
        // use D and E from OptimalSequencePlanner class
        this->visited_berm_count = vector<int>(D, 0);
        this->visited_excavations = vector<bool>(E, false);
        this->current_exc_site = 0;
        this->num_dumps_per_segment = num_dumps_per_segment;
    }

    // Constructor with arguments
    TaskState(vector<int> visited_berm_count, vector<bool> visited_excavations, int current_exc_site, int num_dumps_per_segment)
    {
        this->visited_berm_count = visited_berm_count;
        this->visited_excavations = visited_excavations;
        this->current_exc_site = current_exc_site;
        this->num_dumps_per_segment = num_dumps_per_segment;
    }

    // Equality operator (only check for visited berms and current robot pose, ignore visited excavations)
    bool operator==(const TaskState& other) const
    {
        return (visited_berm_count == other.visited_berm_count && current_exc_site == other.current_exc_site);
    }

    // Print function
    void print()
    {
        cout<<"-------State: "<<endl;
        cout<<"Visited Berm Counts: ";
        for(u_int i=0; i<visited_berm_count.size(); i++)
        {
            cout<<visited_berm_count[i]<<" ";
        }
        cout<<endl;
        cout<<"Visited Excavations: ";
        for(u_int i=0; i<visited_excavations.size(); i++)
        {
            cout<<visited_excavations[i]<<" ";
        }
        cout<<endl;
        cout<<"Current Robot Pose: "<<current_exc_site<<endl;
        cout<<"-------"<<endl;
    }
};

struct StateHasher
{
    std::size_t operator()(const TaskState& state) const
    {
        std::size_t hash = 0;
        // Iterate over the elements of the set and combine their hashes
        for (const auto& element : state.visited_berm_count){
            hash += std::hash<bool>{}(element);
        }
        hash += std::hash<int>{}(state.current_exc_site);
        return hash;
    }
};

struct Action
{
    int dj, pj, ek;
    Action(int dj, int pj, int ek)
    {
        this->dj = dj;
        this->pj = pj;
        this->ek = ek;
    }

    // Print function
    void print()
    {
        cout<<"-------Action: "<<endl;
        cout<<"Build Berm: "<<dj<<" at "<<pj<<endl;
        cout<<"Goto Excavation: "<<ek<<endl;
        cout<<"-------"<<endl;
    }
};

// ----------------- Classes -----------------

class Astar2D
{
public:
    // 2D Astar constants
    static constexpr int NUMOFDIRS = 8;
    static constexpr double MAXCOST = 1e9;
    static constexpr double BERM_AVOID_DIST_M = 0.5;
    static constexpr array<int, 8> dX = {-1, -1, -1,  0,  0,  1, 1, 1};
    static constexpr array<int, 8> dY = {-1,  0,  1, -1,  1, -1, 0, 1};
    static constexpr array<double, 8> movecost = {1.414, 1, 1.414, 1, 1, 1.414, 1, 1.414};
    static constexpr array<int, 8> invDX = {1, 1, 1, 0, 0, -1, -1, -1};
    static constexpr array<int, 8> invDY = {1, 0, -1, 1, -1, 1, 0, -1};

    // define astar arrays as global variables to avoid stack overflow
    vector<vector<bool>> vis;
    vector<vector<int>> parent;
    vector<vector<int>> cost;
    vector<vector<int>> is_obstacle;

    priority_queue<pair<int, Point2D>, vector<pair<int, Point2D>>, compare_pair<Point2D>> pq;
    int x_size, y_size, resolution;

    Point2D start, goal;

    Astar2D(){}

    Astar2D(int x_size, int y_size, int resolution)
    {   
        this->x_size= x_size;
        this->y_size = y_size;
        this->resolution = resolution; // meters per pixel

        //initialize arrays
        this->cost = vector<vector<int>>(x_size+1, vector<int>(y_size+1, MAXCOST)); //distance array
        this->vis = vector<vector<bool>>(x_size+1, vector<bool>(y_size+1, false));   //visited array
        this->parent = vector<vector<int>>(x_size+1, vector<int>(y_size+1, -1));    //parent array (index values of (dX,dY) saved)
        this->is_obstacle = vector<vector<int>>(x_size+1, vector<int>(y_size+1, -1)); // to hash if a point is an obstacle or not

        //priority queue to pop closest element (dist, point_2d)
        this->pq = priority_queue<pair<int, Point2D>, vector<pair<int, Point2D>>, compare_pair<Point2D>>();
    }

    bool isvalid(Point2D point, int x_size, int y_size)
    {
        if(point.x >= 1 && point.x <= x_size && point.y >= 1 && point.y <= y_size)
            return true;
        return false;
    }

    bool check_if_obstacle(const Point2D& point, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berm_counts, const u_int8_t* map, const int collision_thresh)
    {   
        // If already computed, return value
        if (is_obstacle[point.x][point.y] != -1) return is_obstacle[point.x][point.y];

        // Check if it is a map obstacle
        bool result = map[GETMAPINDEX(point.x, point.y, x_size)] < collision_thresh;
        if(result == true)
        {
            is_obstacle[point.x][point.y] = result;
            return result;
        }

        // Check if it is a berm obstacle
        for (u_int i = 0; i < berm_inputs.size(); i++){
            if (visited_berm_counts[i] == 0) continue;
            Pose2D berm_input = berm_inputs[i];
            double distance = sqrt(pow((point.x*resolution - berm_input.x), 2) + pow((point.y*resolution - berm_input.y), 2));
            if (distance < BERM_AVOID_DIST_M) 
            {
                result = true;
                break;
            }
        }
        is_obstacle[point.x][point.y] = result;
        return result;
    }

    double get_octal_distance(const Point2D& start, const Point2D& goal)
    {
        int dx = abs(start.x - goal.x), dy = abs(start.y - goal.y);
        return (int)(1.414*(min(dx, dy))) + (max(dx, dy) - min(dx, dy));
    }

    //TODO: collision checker with respect to the berms built till now
    double get_plan_cost(const Point2D& start, const Point2D& goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berm_counts, const u_int8_t* map, const int collision_thresh)
    {
        //insert start node with distance 0
        pq.push({0, start});
        cost[start.x][start.y] = 0;
        vis[start.x][start.y] = true;
        parent[start.x][start.y] = -1;

        while(!pq.empty())
        {
            Point2D u = pq.top().second;
            pq.pop();

            // Check if we have reached goal
            if(u.x == goal.x && u.y == goal.y)
            {
                return cost[u.x][u.y];
            }
            
            //iterate over all unvisited neighbors of u
            for(int dir = 0; dir < NUMOFDIRS; dir++)
            {
                Point2D v = {u.x + dX[dir], u.y + dY[dir]};
                // if neighbor is valid and not visited
                if(isvalid(v, x_size, y_size) && vis[v.x][v.y] == false)
                {                    
                    // Check if neighbor is an obstacle
                    if (check_if_obstacle(v, berm_inputs, visited_berm_counts, map, collision_thresh)) continue;
                    
                    double v_g = cost[u.x][u.y] + movecost[dir];

                    //if there is a shorter path to v through u, update cost and push to queue
                    if(cost[v.x][v.y] > v_g)
                    {
                        int v_h = get_octal_distance(v, goal);

                        parent[v.x][v.y] = dir;
                        cost[v.x][v.y] = v_g;
                        pq.push({v_g + v_h, v});
                        vis[v.x][v.y] = true;
                    }
                }
            }
        }
        // if no path found, return max cost
        return MAXCOST;
    }
};

class OptimalSequencePlanner
{
public:
    // number of dump locations and excavation locations
    int D, E;
    Pose2D robot_start_pose;
    bool DEBUG = false;
    double EXCAVATION_DIST_M = 1.5; 
    const double TOOL_DISTANCE_TO_DUMP = 0.85;

    // make shared pointers to store references to the map, berm inputs, and excavation poses
    shared_ptr<vector<Pose2D>> berm_inputs, excavation_poses;
    shared_ptr<nav_msgs::msg::OccupancyGrid> map;
    
    // variables for graph search
    vector<TaskState> states;
    vector<int> parents;
    vector<double> g_values;
    vector<bool> visited_states;
    vector<Action> actions_taken; // action taken to reach state i from state parents[i]
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    unordered_map<TaskState, int, StateHasher> state_to_idx;
    
    // Functions
    bool checkIfGoal(const TaskState& state)
    {
        // Check if all berms have visited num_dumps_per_segment times
        for(u_int i=0; i<state.visited_berm_count.size(); i++)
        {
            if(state.visited_berm_count[i]<state.num_dumps_per_segment) return false;
        }
        return true;
    }

    Pose2D getDumpPose(const Pose2D &berm_section, int pj)
    {   
        int sign = (pj==0 ? 1 : -1);
        Pose2D dump_pose;
        // Calculate dump poses on either side of the berm section with TOOL_DISTANCE_TO_DUMP from the berm section center
        double angle = berm_section.theta + sign*M_PI / 2;
        dump_pose.x = berm_section.x + this->TOOL_DISTANCE_TO_DUMP * cos(angle);
        dump_pose.y = berm_section.y + this->TOOL_DISTANCE_TO_DUMP * sin(angle);
        dump_pose.theta = angle - M_PI;
        return dump_pose;
    }

    double get_astar_cost(Pose2D start, Pose2D goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    {
        double resolution = map->info.resolution; // meters per pixel
        Astar2D astar(map->info.width, map->info.height, resolution);
        Point2D start_grid = {(int) round(start.x/resolution), (int) round(start.y/resolution)};
        Point2D goal_grid = {(int) round(goal.x/resolution), (int) round(goal.y/resolution)};
        double move_cost = astar.get_plan_cost(start_grid, goal_grid, berm_inputs, visited_berms, reinterpret_cast<const u_int8_t*>(map->data.data()), 50);
        return move_cost*resolution;
    }

    TaskState get_next_state(const TaskState& state, int dj, int ek)
    {
        // Returns next state and cost of transition
        TaskState new_state = state;
        new_state.visited_berm_count[dj] += 1;
        new_state.visited_excavations[ek] = true;
        new_state.current_exc_site = ek;
        new_state.num_dumps_per_segment = state.num_dumps_per_segment;
        return new_state;
    }

    double get_next_state_cost(const TaskState& state, int dj, int pj, int ek)
    {
        // Compute excavation cost
        Pose2D robot_start_pose = excavation_poses->at(state.current_exc_site);
        Pose2D robot_excavation_end_pose = Pose2D(robot_start_pose.x + EXCAVATION_DIST_M*cos(robot_start_pose.theta), 
                                                  robot_start_pose.y + EXCAVATION_DIST_M*sin(robot_start_pose.theta), 
                                                  robot_start_pose.theta);
        double excavation_cost = EXCAVATION_DIST_M;

        // Compute dump cost to move from robot_excavation_end_pose to berm_input[dj] at pose pj
        vector<int> visited_berm_counts = state.visited_berm_count; visited_berm_counts[dj] += 1;
        Pose2D robot_dump_pose = getDumpPose(berm_inputs->at(dj), pj);
        double dump_cost = get_astar_cost(robot_excavation_end_pose, robot_dump_pose, *berm_inputs, visited_berm_counts);

        // Compute reset cost to move from robot_dump_pose to excavation_poses[ek]
        Pose2D robot_end_pose = excavation_poses->at(ek);
        double reset_cost = get_astar_cost(robot_dump_pose, robot_end_pose, *berm_inputs, visited_berm_counts);
        double total_cost = excavation_cost + dump_cost + reset_cost;

        return total_cost;
    }

    OptimalSequencePlanner(const nav_msgs::msg::OccupancyGrid& map, const vector<Pose2D>& berm_inputs, const vector<Pose2D>& excavation_poses, int num_dumps_per_segment)
    {
        // Assumes that the robot starts from excavation_poses[0]
        this->map = make_shared<nav_msgs::msg::OccupancyGrid>(map);
        this->berm_inputs = make_shared<vector<Pose2D>>(berm_inputs);
        this->excavation_poses = make_shared<vector<Pose2D>>(excavation_poses);
        this->D = berm_inputs.size();
        this->E = excavation_poses.size();

        this->D = berm_inputs.size();
        this->E = excavation_poses.size();
        // Push initial state to queue
        TaskState initial_state(D, E, num_dumps_per_segment);
        initial_state.visited_excavations[0] = true;
        initial_state.current_exc_site = 0;
        states.push_back(initial_state);
        parents.push_back(-1);
        g_values.push_back(0);
        visited_states.push_back(false);
        actions_taken.push_back(Action(-1, -1, -1));
        state_to_idx[initial_state] = 0;
        pq.push({0, 0});
    }

    void update_neighbor(int u, int dj, int pj, int ej)
    {
        // Function adds neighbor to search if it is not already visited or if it has a lower g value
        // Get next state
        auto new_state = get_next_state(states[u], dj, ej);

        // see if new_state was already reached before (new_state_idx = -1 otherwise)
        int new_state_idx = -1;
        if(state_to_idx.find(new_state)!=state_to_idx.end()) new_state_idx = state_to_idx[new_state];
        
        // If new state is already closed, continue
        if(new_state_idx!=-1 && visited_states[new_state_idx]==true) return;

        double g_val = g_values[u]+get_next_state_cost(states[u], dj, pj, ej);

        // If state not in search, add it
        if(new_state_idx==-1)
        {
            // Heuristics 
            double h_val = 0;
            if (DEBUG)
            {
                cout<<"adding new state: "<<states.size()<<endl;
                cout<<"g_val: "<<g_val<<" dj: "<<dj<<" pj: "<<pj<<" ej: "<<ej<<endl;
                new_state.print();
            }

            // Add state to search
            states.push_back(new_state);
            parents.push_back(u);
            actions_taken.push_back(Action(dj, pj, ej));
            g_values.push_back(g_val);
            visited_states.push_back(false);
            state_to_idx[new_state] = states.size()-1;
            pq.push(make_pair(g_val+h_val, states.size()-1));
        }
        else
        {
            // If we reach a state with lower g value, update it
            if (g_val < g_values[new_state_idx])
            {
                // Heuristics 
                double h_val = 0;

                // If we reach a state with lower g value, update it
                g_values[new_state_idx] = g_val;
                parents[new_state_idx] = u;
                actions_taken[new_state_idx] = Action(dj, pj, ej);
                pq.push(make_pair(g_val+h_val, new_state_idx));
                if(DEBUG)
                {
                    cout<<"updating state: "<<new_state_idx<<endl;
                    cout<<"g_val: "<<g_val<<" dj: "<<dj<<" pj: "<<pj<<" ej: "<<ej<<endl;
                    new_state.print();
                }
            }
        }
      
    }

    void get_plan()
    {
        int goal_idx = -100;  
        int itr = 0;
        while(!pq.empty())
        {
            // pop topmost element
            int u = pq.top().second;
            if(DEBUG) cout<<"*************Iteration: "<<itr<<endl;
            if(DEBUG) cout<<"PQ SIZE: "<<pq.size()<<endl;

            if(DEBUG)
            {
                cout<<"Popped: "<<u<<endl;
                states[u].print();
            }
            
            pq.pop();

            // Break if goal is reached
            bool goal_reached = checkIfGoal(states[u]);
            if(goal_reached)
            {
                cout<<"Goal reached"<<endl;
                goal_idx = u;
                break;
            }
            if(visited_states[u]==true) continue;
            visited_states[u] = true;
            itr++;

            // Check if there is an incomplete berm
            int incomplete_berm_idx = -1;
            for(u_int i=0; i<states[u].visited_berm_count.size(); i++)
            {
                if(states[u].visited_berm_count[i]<states[u].num_dumps_per_segment && states[u].visited_berm_count[i]>0)
                {
                    if(incomplete_berm_idx!=-1)
                    {
                        cout<<"ERROR: More than one incomplete berm"<<endl;
                    }
                    incomplete_berm_idx = i;
                }
            }

            vector<int> possible_dj;
            // If there is an incomplete berm, complete it
            if(incomplete_berm_idx!=-1)
            {
                possible_dj.push_back(incomplete_berm_idx);
            }
            else
            {
                // Push all non started berms to possible_dj
                for(u_int i=0; i<states[u].visited_berm_count.size(); i++)
                {
                    if(states[u].visited_berm_count[i]==0)
                    {
                        possible_dj.push_back(i);
                    }
                }
            }

            // expand all possible states from u
            for(int dj: possible_dj)
            {
                for(int ej = 0; ej<E; ej++) // iterate over all excavations
                {   
                    if(states[u].visited_excavations[ej]==true) continue; // if ej has been visited, continue
                    // iterate through all dump poses for dj
                    for(int pj = 0; pj<2; pj++) 
                    {
                        // cout<<"Updating neighbor"<<" u: "<<u<<" dj: "<<dj<<" pj: "<<pj<<" ej: "<<ej<<endl;
                        update_neighbor(u, dj, pj, ej);
                    }
                }                
            }
            if(DEBUG) cout<<"##################"<<endl;
        }

        if (goal_idx == -100) 
        {
            cout<<"No path found"<<endl;
            return;
        }

        // Print path and cost (backtrack from goal)
        int curr_idx = goal_idx;
        vector<int> path;
        while(curr_idx!=-1)
        {
            path.push_back(curr_idx);
            curr_idx = parents[curr_idx];
        }
        reverse(path.begin(), path.end());
       
        // Print path with actions taken
        cout<<"\n\nOptimal Path is: "<<endl;
        for(u_int i=0; i<path.size(); i++)
        {   
            cout<<"\n************ Node IDX: "<<path[i]<<endl;
            cout<<"Cost: "<<g_values[path[i]]<<endl;
            states[path[i]].print();
            if (i<path.size()-1) actions_taken[path[i+1]].print();
            cout<<"************"<<endl;
        }
    }
};

class TaskOptimizer: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        using PlanTask = lx_msgs::action::PlanTask;
        using GoalHandlePlanTask = rclcpp_action::ServerGoalHandle<PlanTask>;

        // Task planner variables
        std::vector<Pose2D> berm_inputs_;
        std::vector<Pose2D> excavation_poses_;
        std::vector<int> berm_section_iterations_;
        std::vector<double> berm_section_heights_;
        double section_length_;
        double desired_berm_height_;
        nav_msgs::msg::OccupancyGrid map_;

        // Servers
        rclcpp_action::Server<PlanTask>::SharedPtr taskplanner_action_server_;

        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Put next function here
        * */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& , std::shared_ptr<const PlanTask::Goal> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action cancel request
        * */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlanTask> );

        /*
        * Argument(s):
        *   - Goal handle shared pointer
        * 
        * Handle action accepted
        * */
        void handle_accepted(const std::shared_ptr<GoalHandlePlanTask> );

        void findBermSequence(const std::shared_ptr<GoalHandlePlanTask> );
        
        std::vector<Pose2D> getDumpPoses(const Pose2D& );

        int numOfDumps(const int );

    public:

        // Berm parameters
        static constexpr double INIT_BERM_HEIGHT = 0.09;            // m
        static constexpr double ANGLE_OF_REPOSE = 30;               // degrees

        // Rover parameters
        static constexpr double ROVER_WIDTH = 0.7;                  // m (actual width is 0.67 m)
        static constexpr double ROVER_LENGTH = 1.0;                 // m (actual length is 0.988 m)
        static constexpr double MAX_TOOL_DISTANCE_FROM_BASE = 1.0;  // m (conservative estimate for collision)
        static constexpr double TOOL_DISTANCE_TO_DUMP = 0.85;        // m

        // Functions
        /*
        * Constructor
        * */
        TaskOptimizer();

        /*
        * Destructor
        * */
        ~TaskOptimizer(){};

        // Function to get rover footprint
        std::vector<geometry_msgs::msg::Point> getRoverFootprint(const Pose2D& );

        // Function to get bounds of rover footprint
        Bounds getBounds(const std::vector<geometry_msgs::msg::Point>& );
};

#endif