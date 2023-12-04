#ifndef TASK_OPTIMIZER_H
#define TASK_OPTIMIZER_H

#include "base.hpp"
#include "edge_cost_search.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/action/plan_task.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lx_msgs/msg/berm_section.hpp"


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
    vector<Pose2D> berm_inputs, excavation_poses;
    nav_msgs::msg::OccupancyGrid map;
    
    // variables for graph search
    vector<TaskState> states;
    vector<int> parents;
    vector<double> g_values;
    vector<bool> visited_states;
    vector<Action> actions_taken; // action taken to reach state i from state parents[i]
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    unordered_map<TaskState, int, StateHasher> state_to_idx;

    OptimalSequencePlanner(const nav_msgs::msg::OccupancyGrid& map, const vector<Pose2D>& berm_inputs, const vector<Pose2D>& excavation_poses, int num_dumps_per_segment)
    {
        // Assumes that the robot starts from excavation_poses[0]
        this->map = map;
        this->berm_inputs = berm_inputs;
        this->excavation_poses = excavation_poses;
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

    Pose2D getDumpPose(const Pose2D &berm_section, const int pj)
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

    double get_astar_cost(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    {
        double resolution = map.info.resolution; // meters per pixel
        Astar2D astar(map.info.width, map.info.height, resolution);
        Point2D start_grid = {(int) round(start.x/resolution), (int) round(start.y/resolution)};
        Point2D goal_grid = {(int) round(goal.x/resolution), (int) round(goal.y/resolution)};
        double move_cost = astar.get_plan_cost(start_grid, goal_grid, berm_inputs, visited_berms, reinterpret_cast<const u_int8_t*>(map.data.data()), 50);
        if (move_cost == DBL_MAX) return DBL_MAX;
        return move_cost * resolution;
    }

    vector<Pose2D> get_astar_path(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    {
        double resolution = map.info.resolution; // meters per pixel
        Astar2D astar(map.info.width, map.info.height, resolution);
        Point2D start_grid = {(int) round(start.x/resolution), (int) round(start.y/resolution)};
        Point2D goal_grid = {(int) round(goal.x/resolution), (int) round(goal.y/resolution)};
        vector<Point2D> path = astar.get_path(start_grid, goal_grid, berm_inputs, visited_berms, reinterpret_cast<const u_int8_t*>(map.data.data()), 50);
        // Multiply path by resolution
        Pose2D pose;
        vector<Pose2D> path_poses;
        for(u_int i=0; i<path.size(); i++)
        {
            pose.x = path[i].x*resolution;
            pose.y = path[i].y*resolution;
            if (i < path.size() - 1)
            {
                pose.theta = atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x);
            }
            path_poses.push_back(pose);
        }
        return path_poses;
    }

    TaskState get_next_state(const TaskState& state, const int &dj, const int &ek)
    {
        // Returns next state and cost of transition
        TaskState new_state = state;
        new_state.visited_berm_count[dj] += 1;
        new_state.visited_excavations[ek] = true;
        new_state.current_exc_site = ek;
        new_state.num_dumps_per_segment = state.num_dumps_per_segment;
        return new_state;
    }

    double get_next_state_cost(const TaskState& state, const int &dj, const int &pj, const int &ek)
    {
        // Compute excavation cost
        Pose2D robot_start_pose = excavation_poses[state.current_exc_site];
        Pose2D robot_excavation_end_pose = Pose2D(robot_start_pose.x + EXCAVATION_DIST_M*cos(robot_start_pose.theta), 
                                                  robot_start_pose.y + EXCAVATION_DIST_M*sin(robot_start_pose.theta), 
                                                  robot_start_pose.theta);
        double excavation_cost = EXCAVATION_DIST_M;

        // Compute dump cost to move from robot_excavation_end_pose to berm_input[dj] at pose pj
        vector<int> visited_berm_counts = state.visited_berm_count; visited_berm_counts[dj] += 1;
        Pose2D robot_dump_pose = getDumpPose(berm_inputs[dj], pj);
        double dump_cost = get_astar_cost(robot_excavation_end_pose, robot_dump_pose, berm_inputs, visited_berm_counts);
        if (dump_cost == DBL_MAX) return DBL_MAX;

        // Compute reset cost to move from robot_dump_pose to excavation_poses[ek]
        Pose2D robot_end_pose = excavation_poses[ek];
        double reset_cost = get_astar_cost(robot_dump_pose, robot_end_pose, berm_inputs, visited_berm_counts);
        if (reset_cost == DBL_MAX) return DBL_MAX;
        double total_cost = excavation_cost + dump_cost + reset_cost;
        return total_cost;
    }

    void update_neighbor(const int &u, const int &dj, const int &pj, const int &ej)
    {
        // Function adds neighbor to search if it is not already visited or if it has a lower g value
        // Get next state
        auto new_state = get_next_state(states[u], dj, ej);

        // see if new_state was already reached before (new_state_idx = -1 otherwise)
        int new_state_idx = -1;
        if(state_to_idx.find(new_state)!=state_to_idx.end()) new_state_idx = state_to_idx[new_state];
        
        // If new state is already closed, continue
        if(new_state_idx!=-1 && visited_states[new_state_idx]==true) return;

        double next_state_cost = get_next_state_cost(states[u], dj, pj, ej);
        if (next_state_cost == DBL_MAX) return;
        
        double g_val = g_values[u] + next_state_cost;

        // If state not in search, add it
        if(new_state_idx==-1)
        {
            // Heuristics 
            double h_val = 0;
            // if (DEBUG)
            // {
            //     cout<<"adding new state: "<<states.size()<<endl;
            //     cout<<"g_val: "<<g_val<<" dj: "<<dj<<" pj: "<<pj<<" ej: "<<ej<<endl;
            //     new_state.print();
            // }

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
                // if(DEBUG)
                // {
                //     cout<<"updating state: "<<new_state_idx<<endl;
                //     cout<<"g_val: "<<g_val<<" dj: "<<dj<<" pj: "<<pj<<" ej: "<<ej<<endl;
                //     new_state.print();
                // }
            }
        }
      
    }

    bool save_path(const vector<Pose2D> &path, const string& filename)
    {
        ofstream myfile;
        myfile.open(filename);
        if(!myfile.is_open())
        {
            cout<<"Error opening file"<<endl;
            return false;
        }
        for(u_int i=0; i<path.size(); i++)
        {
            myfile<<path[i].x<<","<<path[i].y<<","<<path[i].theta<<endl;
        }
        myfile.close();
        return true;
    }

    vector<lx_msgs::msg::PlannedTask> get_plan()
    {
        vector<lx_msgs::msg::PlannedTask> final_plan;
        int goal_idx = -100;  
        int itr = 0;
        while(!pq.empty())
        {
            // pop topmost element
            int u = pq.top().second;
            if(DEBUG) cout<<"*************Iteration: "<<itr<<endl;
            if(DEBUG) cout<<"PQ SIZE: "<<pq.size()<<endl;

            // if(DEBUG)
            // {
            //     cout<<"Popped: "<<u<<endl;
            //     states[u].print();
            // }
            
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
                        if(DEBUG) cout<<"Updating neighbor"<<" u: "<<u<<" dj: "<<dj<<" pj: "<<pj<<" ej: "<<ej<<endl;
                        update_neighbor(u, dj, pj, ej);
                    }
                }
            }
            if(DEBUG) cout<<"##################"<<endl;
        }

        if (goal_idx == -100) 
        {
            cout<<"No path found"<<endl;
            return final_plan;
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
        cout<<"\nOptimal Path is: "<<endl;
        for(u_int i=0; i<path.size(); i++)
        {   
            cout<<"\n************ Node IDX: "<<path[i]<<endl;
            cout<<"Cost: "<<g_values[path[i]]<<endl;
            states[path[i]].print();
            if (i<path.size()-1) actions_taken[path[i+1]].print();
            cout<<"************"<<endl;
        }

        bool first_op = true;
        int nav_count = 0;
        string dir = "/home/lx_autonomy/lx_autonomy_ws/src/lx_packages/lx_planning/paths/";

        vector<int> visited_berm_counts(D, 0);

        Pose2D robot_cur_pose = excavation_poses[0], robot_next_pose;
        lx_msgs::msg::PlannedTask task;
        for (u_int i = 0; i < path.size() -1; i++)
        {
            Action action_to_take = actions_taken[path[i+1]];
            int berm_idx = action_to_take.dj;
            int excavation_idx = action_to_take.ek;

            if (!first_op) {
                // Navigation Task to Excavation
                task.task_type = int(TaskTypeEnum::AUTONAV);
                task.pose = excavation_poses[excavation_idx].getPose();
                final_plan.push_back(task);

                vector<Pose2D> path = get_astar_path(robot_cur_pose, excavation_poses[excavation_idx], berm_inputs, visited_berm_counts);
                robot_cur_pose = excavation_poses[excavation_idx];

                // Save path to file
                save_path(path, dir + "path_" + to_string(nav_count++) + ".txt");
            }
            else {
                first_op = false;
            }

            // Excavation Task
            task.task_type = int(TaskTypeEnum::AUTODIG);
            task.pose = excavation_poses[excavation_idx].getPose();
            final_plan.push_back(task);
            robot_cur_pose = excavation_poses[excavation_idx];
            robot_cur_pose.x += EXCAVATION_DIST_M*cos(robot_cur_pose.theta);
            robot_cur_pose.y += EXCAVATION_DIST_M*sin(robot_cur_pose.theta);

            // Navigation Task to Berm
            task.task_type = int(TaskTypeEnum::AUTONAV);
            Pose2D dump_pose = getDumpPose(berm_inputs[berm_idx], action_to_take.pj);
            task.pose = dump_pose.getPose();
            final_plan.push_back(task);
            vector<Pose2D> path = get_astar_path(robot_cur_pose, dump_pose, berm_inputs, visited_berm_counts);
            robot_cur_pose = dump_pose;

            save_path(path, dir + "path_" + to_string(nav_count++) + ".txt");

            // Dump Task
            task.task_type = int(TaskTypeEnum::AUTODUMP);
            task.pose = berm_inputs[berm_idx].getPose();
            final_plan.push_back(task);

            visited_berm_counts[berm_idx] += 1;
        }

        return final_plan;
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
};

#endif