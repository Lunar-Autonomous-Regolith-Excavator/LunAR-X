#ifndef BASE_H
#define BASE_H

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
#include <fstream>

#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>

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

struct PointMock {
    double x;
    double y;
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
    void print() const
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

class Map2D
{
public:
    static constexpr double UNKNOWN = 255;
	static constexpr double OCCUPIED = 254;
	static constexpr double INSCRIBED = 253;
	static constexpr double FREE = 0;

	cv::Mat data;
	double resolution;
	double origin_x;
	double origin_y;
	double size_x;
	double size_y;

    Map2D() {}

    Map2D(double resolution, double origin_x, double origin_y, double size_x, double size_y) {
        this->resolution = resolution;
        this->origin_x = origin_x;
        this->origin_y = origin_y;
        this->size_x = size_x;
        this->size_y = size_y;
    }

    Map2D(cv::Mat data) {
        this->data = data;
    }

    Map2D(nav_msgs::msg::OccupancyGrid map) {
        this->resolution = map.info.resolution;
        this->origin_x = map.info.origin.position.x;
        this->origin_y = map.info.origin.position.y;
        this->size_x = map.info.width;
        this->size_y = map.info.height;

        // Convert to cv::Mat
        cv::Mat img(map.info.height, map.info.width, CV_8UC1);
        for (uint i = 0; i < map.info.height; i++) {
            for (uint j = 0; j < map.info.width; j++) {
                int idx = GETMAPINDEX(j, i, map.info.width);
                if (static_cast<int>(map.data[idx]) < 50) {
                    img.at<uint8_t>(i, j) = FREE;
                } else {
                    img.at<uint8_t>(i, j) = OCCUPIED;
                }
            }
        }
        // Flip image
        cv::flip(img, img, 0);
        
        this->data = img;
    }

    void mark_all_free() {
        for (uint i = 0; i < this->size_y; i++) {
            for (uint j = 0; j < this->size_x; j++) {
                this->data.at<uint8_t>(i, j) = FREE;
            }
        }
    }

	bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
	{
	  if (wx < origin_x || wy < origin_y)
		 return false;
		 
	  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
	  my = static_cast<unsigned int>((wy - origin_y) / resolution);

	  if (mx < size_x && my < size_y)
		 return true;

	  return false;
	}

	void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
	{
		wx = origin_x + (mx + 0.5) * resolution;
		wy = origin_y + (my + 0.5) * resolution;		
	}

	double getCost(int x, int y)
	{		
		return data.at<uint8_t>(y, x);
	}

	double getCost(int idx)
	{
		int x = idx % getSizeInCellsX();
		int y = idx / getSizeInCellsY();
		
		return getCost(x, y);
	}

	unsigned int getSizeInCellsX()
	{
		return size_x;
	}
	
	unsigned int getSizeInCellsY()
	{
		return size_y;
	}

    void display() {
        cv::imshow("map", this->data);
        cv::waitKey(0);
    }
};

#endif // BASE_H