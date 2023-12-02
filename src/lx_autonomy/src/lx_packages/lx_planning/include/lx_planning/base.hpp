#ifndef BASE_H
#define BASE_H

#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/utils.h>

#include <vector>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <fstream>

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

    geometry_msgs::msg::Pose getPose() {
        geometry_msgs::msg::Pose pose;
        pose.position.x = this->x;
        pose.position.y = this->y;
        pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, this->theta);
        pose.orientation = tf2::toMsg(q);
        return pose;
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

#endif // BASE_H