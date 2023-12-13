#ifndef EDGE_COST_SEARCH_H
#define EDGE_COST_SEARCH_H

#include "base.hpp"
#include "climits"

// ----------------- Classes -----------------

class Astar2D
{
public:
    // 2D Astar constants
    static constexpr int NUMOFDIRS = 8;
    static constexpr double BERM_AVOID_DIST_M = 0.25;
    static constexpr double ROBOT_RADIUS_M = 0.5;

    static constexpr array<int, 8> dX = {-1, -1, -1,  0,  0,  1, 1, 1};
    static constexpr array<int, 8> dY = {-1,  0,  1, -1,  1, -1, 0, 1};
    static constexpr array<double, 8> movecost = {1.414, 1, 1.414, 1, 1, 1.414, 1, 1.414};
    static constexpr array<int, 8> invDX = {1, 1, 1, 0, 0, -1, -1, -1};
    static constexpr array<int, 8> invDY = {1, 0, -1, 1, -1, 1, 0, -1};

    // define astar arrays as global variables to avoid stack overflow
    vector<vector<bool>> vis;
    vector<vector<int>> parent;
    vector<vector<double>> cost;
    vector<vector<int>> is_obstacle;

    priority_queue<pair<double, Point2D>, vector<pair<double, Point2D>>, compare_pair<Point2D>> pq;
    int x_size, y_size; 
    double resolution;

    Point2D start, goal;

    Astar2D(){}

    Astar2D(int x_size, int y_size, double resolution)
    {   
        this->x_size= x_size;
        this->y_size = y_size;
        this->resolution = resolution; // meters per pixel

        //initialize arrays
        this->cost = vector<vector<double>>(x_size+1, vector<double>(y_size+1, DBL_MAX)); //distance array
        this->vis = vector<vector<bool>>(x_size+1, vector<bool>(y_size+1, false));   //visited array
        this->parent = vector<vector<int>>(x_size+1, vector<int>(y_size+1, -1));    //parent array (index values of (dX,dY) saved)
        this->is_obstacle = vector<vector<int>>(x_size+1, vector<int>(y_size+1, -1)); // to hash if a point is an obstacle or not

        //priority queue to pop closest element (dist, point_2d)
        this->pq = priority_queue<pair<double, Point2D>, vector<pair<double, Point2D>>, compare_pair<Point2D>>();
    }

    bool isvalid(Point2D point, int x_size, int y_size)
    {
        if(point.x >= 1 && point.x <= x_size && point.y >= 1 && point.y <= y_size)
            return true;
        return false;
    }
    bool check_if_obstacle(const Point2D& point, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berm_counts, const nav_msgs::msg::OccupancyGrid& map, const int collision_thresh)
    {   
        // If already computed, return value
        if (is_obstacle[point.x][point.y] != -1) return is_obstacle[point.x][point.y];

        // Check if it is a map obstacle
        bool result = map.data[point.x*map.info.height + point.y] > collision_thresh;
        if(result == true)
        {
            is_obstacle[point.x][point.y] = result;
            return result;
        }

        // Check if it is a map obstacle within ROBOT_RADIUS_M
        bool is_obstacle_within_radius = false;
        int min_left = std::floor(-ROBOT_RADIUS_M/resolution), max_right = std::ceil(ROBOT_RADIUS_M/resolution);
        for (int i = min_left; i <= max_right; i++){
            for (int j = min_left; j <= max_right; j++)
            {
                if (isvalid({point.x + i, point.y + j}, map.info.width, map.info.height)
                    && map.data[(point.x + i)*map.info.height + point.y + j] > collision_thresh)
                {
                    is_obstacle_within_radius = true;
                    break;
                }
            }
            if (is_obstacle_within_radius) break;
        }
        if (is_obstacle_within_radius)
        {
            is_obstacle[point.x][point.y] = true;
            return true;
        }

        // Check if it is a berm obstacle
        for (u_int i = 0; i < berm_inputs.size(); i++){
            if (visited_berm_counts[i] == 0) continue;
            Pose2D berm_input = berm_inputs[i];
            double point_x = point.x, point_y = point.y;
            point_x *= resolution; point_y *= resolution;
            double distance = sqrt(pow(point_x - berm_input.x, 2) + pow(point_y - berm_input.y, 2));
            if (distance < (BERM_AVOID_DIST_M + ROBOT_RADIUS_M))
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
    double get_plan_cost(const Point2D& start, const Point2D& goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berm_counts, const nav_msgs::msg::OccupancyGrid& map, const int collision_thresh)
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
        return DBL_MAX;
    }

    vector<Point2D> get_path(const Point2D& start, const Point2D& goal, const vector<Pose2D>& berm_inputs, const vector<int>& visited_berm_counts, const nav_msgs::msg::OccupancyGrid& map, const int collision_thresh)
    {
        vector<Point2D> path;

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
                Point2D curr = goal;
                while(parent[curr.x][curr.y] != -1)
                {
                    path.push_back(curr);
                    int dir = parent[curr.x][curr.y];
                    curr = {curr.x + invDX[dir], curr.y + invDY[dir]}; 
                }
                reverse(path.begin(), path.end());
                return path;
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
        return path;        
    }
};

#endif // EDGE_COST_SEARCH_H