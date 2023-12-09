#ifndef TASK_OPTIMIZER_H
#define TASK_OPTIMIZER_H

#include "base.hpp"
#include "edge_cost_search.hpp"
#include "tsp_solver.hpp"

#include "collision_checker.hpp"
#include "a_star.hpp"
// #include "smoother.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lx_library/lx_utils.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "lx_msgs/action/plan_task.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lx_msgs/msg/berm_section.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <chrono>

using namespace fcc;
using namespace std;
using namespace nav2_smac_planner;

class OptimalSequencePlanner
{
public:
    // number of dump locations and excavation locations
    int D, E;
    int num_dumps_per_segment;
    Pose2D robot_start_pose;
    bool DEBUG = false;
    double EXCAVATION_DIST_M = 1.5; 
    const double TOOL_DISTANCE_TO_DUMP = 0.85;
    const double HEURISTIC_RESOLUTION = 0.05; // Costs are divided by this value before sending to TSP solver
    const bool USE_TSP_HEURISTIC = true;
    const double HEURISTIC_WEIGHT = 100;
    const int COLLISION_THRESH = 50; // grid value below which a cell is considered an obstacle, range -128 to 127

    // make shared pointers to store references to the map, berm inputs, and excavation poses
    vector<Pose2D> berm_inputs, excavation_poses;
    nav_msgs::msg::OccupancyGrid map;
    unsigned int berm_length;
    unsigned int berm_width;
    
    // Hybrid A* variables
    Map2D *map_2d;
    FootprintCollisionChecker<Map2D, PointMock>::Footprint footprint;
    SearchInfo info;
    unsigned int size_theta = 72;

    // Vector of costmap for each berm input
    vector<cv::Mat> berm_costmaps;
    
    // variables for graph search
    vector<TaskState> states;
    vector<int> parents;
    vector<double> g_values;
    vector<bool> visited_states;
    vector<Action> actions_taken; // action taken to reach state i from state parents[i]
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    unordered_map<TaskState, int, StateHasher> state_to_idx;
    GoogleTSPSolver tsp_solver;

    OptimalSequencePlanner(const nav_msgs::msg::OccupancyGrid& map, const vector<Pose2D>& berm_inputs, const vector<Pose2D>& excavation_poses, int num_dumps_per_segment, double berm_length, double berm_height)
    {   
        // Assumes that the robot starts from excavation_poses[0]
        this->map = map;
        this->berm_inputs = berm_inputs;
        this->excavation_poses = excavation_poses;
        this->D = berm_inputs.size();
        this->E = excavation_poses.size();
        this->num_dumps_per_segment = num_dumps_per_segment;
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

        // Hybrid A*
        this->map_2d = new Map2D(map);
        this->footprint.push_back({0.5, 0.35});
        this->footprint.push_back({-0.5, 0.35});
        this->footprint.push_back({-0.5, -0.35});
        this->footprint.push_back({0.5, -0.35});
        this->info.change_penalty = 1.2;
        this->info.non_straight_penalty = 1.4;
        this->info.reverse_penalty = 2.1;
        this->info.minimum_turning_radius = static_cast<unsigned int>(1.0 / map.info.resolution);

        // Berm parameters for berm cost map
        this->berm_length = static_cast<unsigned int>(berm_length / map.info.resolution);
        this->berm_width = static_cast<unsigned int>((2 * (berm_height / tan(M_PI / 6))) / map.info.resolution);
        // Make berm parameters even
        if (this->berm_length % 2 == 1) this->berm_length += 1;
        if (this->berm_width % 2 == 1) this->berm_width += 1;

        generateBermCostmaps();
    }

    cv::Mat generateBerm(const Pose2D &berm_input)
    {
        using namespace cv;

        unsigned int box_length = ceil(sqrt(this->berm_length * this->berm_length + this->berm_width * this->berm_width)) + 2;
        // make odd
        if (box_length % 2 == 0) box_length += 1;

        Mat berm(box_length, box_length, CV_8UC1, cv::Scalar(0));
        int center = (box_length - 1) / 2;
        double angle = -berm_input.theta;

        // Make all points inside rectangle occupied
        for (int i = 0; i < box_length; ++i) {
            for (int j = 0; j < box_length; ++j) {
                int y = i - center;
                int x = j - center;
                if (abs(y) <= this->berm_width / 2 && abs(x) <= this->berm_length / 2) {
                    berm.at<char>(i, j) = Map2D::OCCUPIED;
                }
            }
        }

        Mat rot_berm(box_length, box_length, CV_8UC1, cv::Scalar(0));
        Mat rot_mat = getRotationMatrix2D(Point2f(center, center), angle * 180 / M_PI, 1);
        warpAffine(berm, rot_berm, rot_mat, rot_berm.size());

        // Make mat of map size and place berm in its position
        Mat map_mat(this->map.info.height, this->map.info.width, CV_8UC1, cv::Scalar(0));

        int berm_center_x = static_cast<int>(berm_input.x / this->map.info.resolution);
        int berm_center_y = static_cast<int>(berm_input.y / this->map.info.resolution);
        
        for (int i = 0; i < box_length; ++i) {
            for (int j = 0; j < box_length; ++j) {
                int y = i - center + berm_center_y;
                int x = j - center + berm_center_x;
                if (y >= 0 && y < this->map.info.height && x >= 0 && x < this->map.info.width) {
                    map_mat.at<char>(y, x) = rot_berm.at<char>(i, j);
                }
            }
        }

        // cv::Mat img;
        // resize(map_mat, img, cv::Size(), 10, 10, cv::INTER_NEAREST);
        // img = ~img;
        // cv::imshow("map", img);

	    // if (cv::waitKey(0) == 27)
        //     cv::destroyAllWindows();
        
        return map_mat;
    }

    void generateBermCostmaps()
    {
        for (u_int i = 0; i < berm_inputs.size(); ++i)
        {
            berm_costmaps.push_back(generateBerm(berm_inputs[i]));
        }
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

    void call_hybrid_astar(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms, double &cost, vector<Pose2D> &path_world)
    {
        cv::Mat map_iter = this->map_2d->data;
    
        // Loop through visited berms
        // for (int i = 0; i < visited_berms.size(); ++i)
        // {
        //     if (visited_berms[i] == 0) continue;
        //     cv::Mat berm_costmap = this->berm_costmaps[i];
        //     // Add berm to map
        //     cv::add(map_iter, berm_costmap, map_iter);
        // }

        // Limit map to 0-254
        cv::threshold(map_iter, map_iter, COLLISION_THRESH, Map2D::OCCUPIED, cv::THRESH_BINARY);

        // cv::Mat img;
        // resize(map_iter, img, cv::Size(), 10, 10, cv::INTER_NEAREST);
        // img = ~img;
        // cv::imshow("map", img);

	    // if (cv::waitKey(0) == 27)
        //     cv::destroyAllWindows();

        this->map_2d->data = map_iter;

        AStarAlgorithm<Map2D, GridCollisionChecker<Map2D, PointMock>> a_star(nav2_smac_planner::MotionModel::REEDS_SHEPP, this->info);
        int max_iterations = 1000;
        int it_on_approach = 100;

        unsigned int start_x, start_y, start_theta, goal_x, goal_y, goal_theta;
        if (!this->map_2d->worldToMap(start.x, start.y, start_x, start_y)) return;
        if (!this->map_2d->worldToMap(goal.x, goal.y, goal_x, goal_y)) return;
        
        double theta_p = start.theta;
        if (theta_p < 0) theta_p += 2 * M_PI;
        // cout << "theta_p: " << theta_p << endl;
        start_theta = static_cast<unsigned int>(theta_p / (2 * M_PI) * this->size_theta);
        theta_p = goal.theta;
        if (theta_p < 0) theta_p += 2 * M_PI;
        // cout << "theta_p: " << theta_p << endl;
        goal_theta = static_cast<unsigned int>(theta_p / (2 * M_PI) * this->size_theta);

        if (DEBUG) {
            cout << "world start: " << start.x << " " << start.y << " " << start.theta << endl;
            cout << "start: " << start_x << " " << start_y << " " << start_theta << endl;
            cout << "world goal: " << goal.x << " " << goal.y << " " << goal.theta << endl;
            cout << "goal: " << goal_x << " " << goal_y << " " << goal_theta << endl;
        }

        a_star.initialize(false, max_iterations, it_on_approach);
        a_star.setFootprint(footprint, false);
        a_star.createGraph(map_2d->getSizeInCellsX(), map_2d->getSizeInCellsY(), size_theta, map_2d);
        a_star.setStart(start_x, start_y, start_theta);
        a_star.setGoal(goal_x, goal_y, goal_theta);

        NodeSE2::CoordinateVector path;

        float tolerance = 5.0;
        int num_it = 0;
        bool found = false;
        cost = DBL_MAX;

        try {
            found = a_star.createPath(path, num_it, tolerance, cost);
        }
        catch (const std::runtime_error & e)
        {
            if (DEBUG) cerr << "failed to plan: " << e.what() << endl;
            cost = DBL_MAX;
            return;
        }

        if (DEBUG) {
            cout << "found path: " << found << endl;
            cout << "num_it " << num_it << " of max " << max_iterations << endl;
            cout << "path size " << path.size() << endl;
            cout << "cost " << cost << endl;
        }

        if (!found) return;

        // Convert to world coordinates
        path_world.clear();

        for (int i = path.size() - 1; i >= 0; --i)
        {
            double wx, wy;
            map_2d->mapToWorld(path[i].x, path[i].y, wx, wy);
            double wtheta = path[i].theta / size_theta * 2 * M_PI;
            path_world.push_back(Pose2D(wx, wy, wtheta));
        }
        
        // cv::Mat img = map_2d->data.clone();
        // threshold(img, img, 127, 254, cv::THRESH_BINARY);

        // img = ~img;

        // for (size_t i = 0; i != path_world.size(); ++i) {
        //     unsigned int x, y;
        //     map_2d->worldToMap(path_world[i].x(), path_world[i].y(), x, y);
        //     cv::circle(img, cv::Point(x, y), 5, 127, 1);
        // }

        // cv::imshow("map", img);

	    // if (cv::waitKey(0) == 27)
        //     cv::destroyAllWindows();
    }

    double get_astar_cost(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    {
        double cost = DBL_MAX;
        vector<Pose2D> path;
        call_hybrid_astar(start, goal, berm_inputs, visited_berms, cost, path);
        return cost;
    }

    // double get_astar_cost(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    // {
    //     double resolution = map.info.resolution; // meters per pixel
    //     Astar2D astar(map.info.width, map.info.height, resolution);
    //     Point2D start_grid = {(int) round(start.x/resolution), (int) round(start.y/resolution)};
    //     Point2D goal_grid = {(int) round(goal.x/resolution), (int) round(goal.y/resolution)};
    //     double move_cost = astar.get_plan_cost(start_grid, goal_grid, berm_inputs, visited_berms, map, COLLISION_THRESH);
    //     if (move_cost == DBL_MAX) return DBL_MAX;
    //     return move_cost * resolution;
    // }

    vector<Pose2D> get_astar_path(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    {
        double cost;
        vector<Pose2D> path;
        call_hybrid_astar(start, goal, berm_inputs, visited_berms, cost, path);
        return path;
    }

    // vector<Pose2D> get_astar_path(const Pose2D &start, const Pose2D &goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berms)
    // {
    //     double resolution = map.info.resolution; // meters per pixel
    //     Astar2D astar(map.info.width, map.info.height, resolution);
    //     Point2D start_grid = {(int) round(start.x/resolution), (int) round(start.y/resolution)};
    //     Point2D goal_grid = {(int) round(goal.x/resolution), (int) round(goal.y/resolution)};
    //     vector<Point2D> path = astar.get_path(start_grid, goal_grid, berm_inputs, visited_berms, map, COLLISION_THRESH);
    //     // Multiply path by resolution
    //     Pose2D pose;
    //     vector<Pose2D> path_poses;
    //     for(u_int i=0; i<path.size(); i++)
    //     {
    //         pose.x = path[i].x*resolution;
    //         pose.y = path[i].y*resolution;
    //         pose.theta = 0;
    //         if (i == path.size()-1)
    //         {
    //             pose.theta = goal.theta;
    //         }
    //         path_poses.push_back(pose);
    //     }
    //     return path_poses;
    // }

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

    double get_octal_distance(const Pose2D& start, const Pose2D& goal)
    {
        double dx = abs(start.x - goal.x), dy = abs(start.y - goal.y);
        return (1.414*(min(dx, dy))) + (max(dx, dy) - min(dx, dy));
    }

    double compute_min_octal_cost_pose(const Pose2D& curr_pose, const Pose2D& final_pose, const vector<bool>& visited_excavations)
    {
        // Compute the min cost to go from curr_pose to final_pose through all unvisited excavations
        double min_cost = DBL_MAX;
        for(u_int i=0; i<visited_excavations.size(); i++)
        {
            if(visited_excavations[i]==true) continue;
            Pose2D excavation_pose = excavation_poses[i];
            double cost = get_octal_distance(curr_pose, excavation_pose) + get_octal_distance(excavation_pose, final_pose);
            if(cost<min_cost) min_cost = cost;
        }
        return min_cost;
    }

    double compute_min_octal_cost(const int di, const int dj, const vector<bool> & visited_excavations)
    {
        // Compute the min cost to go from dump site di to dump site dj through all unvisited excavations
        // Takes min across all possible dump poses for di and dj
        double min_cost = DBL_MAX;
        for(int pi=0; pi<=1; pi++)
        {
            for(int pj=0; pj<=1; pj++)
            {
                Pose2D start_pose = getDumpPose(berm_inputs[di], pi);
                Pose2D end_pose = getDumpPose(berm_inputs[dj], pj);
                double cost = compute_min_octal_cost_pose(start_pose, end_pose, visited_excavations);
                if(cost<min_cost) min_cost = cost;
            } 
        }
    }

    double compute_TSP_heuristic(const TaskState& state, bool tsp_debug=false)
    {
        if (USE_TSP_HEURISTIC == false) return 0;
        // Computes an underestimate for the cost to visit all remaining dump zones starting from the current excavation zone
        // Assumes that the unvisited excavation zones can be used any number of times

        // Iterate through all unvisited excavation zones
        // Add the node idx to unvisited_nodes_idx K times, where K is the number of times the node has to be visited 
        if(tsp_debug==true)
        {
            cout<<"\nComputing TSP Heuristic from:"<<endl;
            state.print();   
        }
        vector<int> unvisited_nodes_idx;
        for(u_int i=0; i<state.visited_berm_count.size(); i++)
        {
            // Add the node idx to unvisited_nodes_idx K times, where K is the number of times the node has to be visited
            for(int j=0; j<state.num_dumps_per_segment-state.visited_berm_count[i]; j++)
            {
                unvisited_nodes_idx.push_back(i);
            }
        }

        if(tsp_debug==true)
        {
            cout<<"Unvisited Nodes: ";
            for(u_int i=0; i<unvisited_nodes_idx.size(); i++)
            {
                cout<<unvisited_nodes_idx[i]<<" ";
            }
            cout<<endl;
        }

        // Compute costs from each unvisited node to all other unvisited nodes
        // Extra entry for excavation zone (last row and column)
        vector<vector<double>> cost_matrix(unvisited_nodes_idx.size()+1, vector<double>(unvisited_nodes_idx.size()+1, 0)); // Convention cost_matrix[from_node][to_node] = cost

        for (u_int i = 0; i < unvisited_nodes_idx.size() + 1; i++)
        {
            for (u_int j = 0; j < unvisited_nodes_idx.size() + 1; j++)
            {
                if (i == j) continue;
                if (i == unvisited_nodes_idx.size()) // start excavation zone to each unvisited_node_idx
                {
                    double min_cost = DBL_MAX;
                    Pose2D start_pose = excavation_poses[state.current_exc_site];
                    for(int p=0; p<=1; p++)
                    {
                        Pose2D end_pose = getDumpPose(berm_inputs[unvisited_nodes_idx[j]], p);
                        double cost = get_octal_distance(start_pose, end_pose);
                        if(cost<min_cost) min_cost = cost;
                    }
                    cost_matrix[i][j] = min_cost;
                }
                else if (j == unvisited_nodes_idx.size()) // terminal cost to go to any excavation zone = 0
                {
                    cost_matrix[i][j] = 0;
                }
                else
                {
                    // Compute min octal cost from unvisited_nodes_idx[i] to unvisited_nodes_idx[j]
                    int di = unvisited_nodes_idx[i], dj = unvisited_nodes_idx[j];
                    double cost = compute_min_octal_cost(di, dj, state.visited_excavations);
                    cost_matrix[i][j] = cost;
                }
            }
        }

        if(tsp_debug==true)
        {
            cout<<"Cost Matrix: "<<endl;
            for(u_int i=0; i<cost_matrix.size(); i++)
            {
                for(u_int j=0; j<cost_matrix.size(); j++)
                {
                    cout<<cost_matrix[i][j]<<" ";
                }
                cout<<endl;
            }
        }

        // Convert cost matrix to int using HEURISTIC_RESOLUTION
        vector<vector<int64_t>> tsp_cost_matrix(cost_matrix.size(), vector<int64_t>(cost_matrix.size(), 0));
        for (u_int i = 0; i < cost_matrix.size(); i++)
        {
            for (u_int j = 0; j < cost_matrix.size(); j++)
            {
                tsp_cost_matrix[i][j] = static_cast<int64_t>(cost_matrix[i][j] / HEURISTIC_RESOLUTION);
            }
        }

        if(tsp_debug==true)
        {
            cout<<"TSP Cost Matrix: "<<endl;
            for(u_int i=0; i<tsp_cost_matrix.size(); i++)
            {
                for(u_int j=0; j<tsp_cost_matrix.size(); j++)
                {
                    cout<<tsp_cost_matrix[i][j]<<" ";
                }
                cout<<endl;
            }
        }

        // Call TSP solver and get cost
        int final_cost = tsp_solver.solve(tsp_cost_matrix, unvisited_nodes_idx.size());

        if (tsp_debug==true)
        {
            cout<<"Final Cost: "<<final_cost<<endl;
        }

        // Multiply final_cost by HEURISTIC_RESOLUTION
        double final_cost_double = (double)final_cost * HEURISTIC_RESOLUTION;

        if (tsp_debug==true)
        {
            cout<<"Final Cost Double: "<<final_cost_double<<endl;
        }

        return final_cost_double*HEURISTIC_WEIGHT;
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
            double h_val = compute_TSP_heuristic(new_state, false);
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
                double h_val = compute_TSP_heuristic(new_state, false);

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
        // Check if num dumps < num available excavations
        int num_dumps_required = berm_inputs.size() * this->num_dumps_per_segment;
        int num_excavations = excavation_poses.size();
        if (num_dumps_required > num_excavations)
        {
            cout<<"ERROR: Number of dumps required is greater than number of excavations available"<<endl;
            return vector<lx_msgs::msg::PlannedTask>();
        }

        auto init_time = chrono::high_resolution_clock::now();
        vector<lx_msgs::msg::PlannedTask> final_plan;
        int goal_idx = -100;  
        int itr = 0;
        int min_num_berms_to_build= INT_MAX;
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
            // after every 20 itrs, print num states required to flip
            int num_berms_to_build = 0;
            for(u_int i=0; i<states[u].visited_berm_count.size(); i++)
            {
                num_berms_to_build += states[u].num_dumps_per_segment - states[u].visited_berm_count[i];
            }
            min_num_berms_to_build = min(min_num_berms_to_build, num_berms_to_build);
            cout<<"Min num berms to build: "<<min_num_berms_to_build<<" Nodes in PQ: "<<pq.size()<<endl;
            
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
        auto final_time = chrono::high_resolution_clock::now();

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

        // Print Solution Stats:
        cout<<"\nSolution Stats: "<<endl;
        cout<<"Heuristic Used: "<<(USE_TSP_HEURISTIC ? "TSP" : "None")<<endl;
        cout<<"Heuristic Weight: "<<HEURISTIC_WEIGHT<<endl;
        cout<<"Num Nodes Generated: "<<states.size()<<endl;
        cout<<"Solution Cost: "<<g_values[goal_idx]<<endl;
        cout<<"Time (s)"<< chrono::duration_cast<chrono::milliseconds>(final_time - init_time).count()/1000.0<<endl;

        int nav_count = 0;
        string dir = ament_index_cpp::get_package_share_directory("lx_planning") + "/paths/";

        vector<int> visited_berm_counts(D, 0);

        // Excavation Task
        lx_msgs::msg::PlannedTask task;
        task.task_type = int(TaskTypeEnum::AUTODIG);
        task.pose = excavation_poses[0].getPose();
        final_plan.push_back(task);
        Pose2D robot_cur_pose = excavation_poses[0];
        robot_cur_pose.x += EXCAVATION_DIST_M*cos(robot_cur_pose.theta);
        robot_cur_pose.y += EXCAVATION_DIST_M*sin(robot_cur_pose.theta);

        for (u_int i = 0; i < path.size() -1; i++)
        {
            Action action_to_take = actions_taken[path[i+1]];
            int berm_idx = action_to_take.dj;
            int excavation_idx = action_to_take.ek;

            // Navigation Task to Berm
            task.task_type = int(TaskTypeEnum::AUTONAV);
            Pose2D dump_pose = getDumpPose(berm_inputs[berm_idx], action_to_take.pj);
            task.pose = dump_pose.getPose();
            final_plan.push_back(task);
            vector<Pose2D> nav_path = get_astar_path(robot_cur_pose, dump_pose, berm_inputs, visited_berm_counts);
            robot_cur_pose = dump_pose;

            if (nav_path.size() == 0) {
                cout << "ERROR: No path found for " << nav_count + 1 << endl;
            }

            save_path(nav_path, dir + "path_" + to_string(nav_count++) + ".txt");

            // Dump Task
            task.task_type = int(TaskTypeEnum::AUTODUMP);
            task.pose = berm_inputs[berm_idx].getPose();
            final_plan.push_back(task);

            // Navigation Task to Excavation
            task.task_type = int(TaskTypeEnum::AUTONAV);
            task.pose = excavation_poses[excavation_idx].getPose();
            final_plan.push_back(task);

            nav_path.clear();
            nav_path = get_astar_path(robot_cur_pose, excavation_poses[excavation_idx], berm_inputs, visited_berm_counts);
            robot_cur_pose = excavation_poses[excavation_idx];

            if (nav_path.size() == 0) {
                cout << "ERROR: No path found for " << nav_count + 1 << endl;
            }

            save_path(nav_path, dir + "path_" + to_string(nav_count++) + ".txt");

            if (i == path.size() - 2) break;
            // Excavation Task
            task.task_type = int(TaskTypeEnum::AUTODIG);
            task.pose = excavation_poses[excavation_idx].getPose();
            final_plan.push_back(task);
            robot_cur_pose = excavation_poses[excavation_idx];
            robot_cur_pose.x += EXCAVATION_DIST_M*cos(robot_cur_pose.theta);
            robot_cur_pose.y += EXCAVATION_DIST_M*sin(robot_cur_pose.theta);

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