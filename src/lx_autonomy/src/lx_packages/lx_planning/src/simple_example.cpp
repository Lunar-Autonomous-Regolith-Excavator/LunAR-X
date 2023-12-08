// Copyright (c) 2021 Bartosz Meglicki <meglickib@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <iostream>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "lx_planning/collision_checker.hpp"
#include "lx_planning/a_star.hpp"
#include "lx_planning/smoother.hpp"

struct PointMock
{
	double x;
	double y;
};

using namespace fcc;
using namespace std;
using namespace nav2_smac_planner;

class Map2D
{
public:
	bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
	{
	  if (wx < 0 || wy < 0)
		 return false;
 
	  mx = static_cast<unsigned int>(wx / resolution);
	  my = static_cast<unsigned int>(wy / resolution);

	  if (mx < size_x && my < size_y)
		 return true;

	  return false;
	}

	void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
	{
		wx = (mx + 0.5) * resolution;
		wy = (my + 0.5) * resolution;
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

	bool fromImage(const std::string &filename)
	{
		using namespace cv;

		data = imread(filename, IMREAD_GRAYSCALE);

		if(data.empty())
			return false;

		size_x = data.cols;
		size_y = data.rows;

		threshold(data, data, 127, 254, THRESH_BINARY_INV);

		return true;
	}

	const cv::Mat &getData()
	{
		return data;
	}

	static constexpr double UNKNOWN = 255;
	static constexpr double OCCUPIED = 254;
	static constexpr double INSCRIBED = 253;
	static constexpr double FREE = 0;

private:
	cv::Mat data;
	double size_x;
	double size_y;
	double resolution = 0.05;
};

void Display(Map2D *map,
             const std::vector<Eigen::Vector2d> &pathWorld)
{
	cv::Mat img = map->getData().clone();
	// cv::resize(map->getData(), img, cv::Size(map->getSizeInCellsX(), map->getSizeInCellsY()));
	threshold(img, img, 127, 254, cv::THRESH_BINARY);

	img = ~img;

	for (size_t i = 0; i != pathWorld.size(); ++i) {
		// cv::circle(img, cv::Point(pathWorld[i].x(), pathWorld[i].y()), 5, 127, 1);
		unsigned int x, y;
		map->worldToMap(pathWorld[i].x(), pathWorld[i].y(), x, y);
		cv::circle(img, cv::Point(x, y), 5, 127, 1);
	}

	cv::imshow("map", img);
}

int main(int argc, char **argv)
{
	Map2D *map = new Map2D();

	if(!map->fromImage("/home/hariharan/ros_ws/src/lx_planning/maps/moonyard.png"))
	{
		cerr << "failed to load map, terminating" << endl;
		return 1;
	}
	
	FootprintCollisionChecker<Map2D, PointMock>::Footprint footprint;

	// In world coordinates
	footprint.push_back({0.5, 0.35});
	footprint.push_back({-0.5, 0.35});
	footprint.push_back({-0.5, -0.35});
	footprint.push_back({0.5, -0.35});

	SearchInfo info;

	info.change_penalty = 1.2;
	info.non_straight_penalty = 1.4;
	info.reverse_penalty = 2.1;
	info.minimum_turning_radius = 2.5 / 0.05;

	unsigned int size_theta = 72;

	cv::namedWindow("map", 1);

	AStarAlgorithm<Map2D, GridCollisionChecker<Map2D, PointMock>> a_star(nav2_smac_planner::MotionModel::REEDS_SHEPP, info);
	int max_iterations = 100000;
	int it_on_approach = 100;

	cv::Point start = cv::Point(2, 4);
	cv::Point end = cv::Point(5, 4);
	unsigned int start_x, start_y;
	map->worldToMap(start.x, start.y, start_x, start_y);
	unsigned int end_x, end_y;
	map->worldToMap(end.x, end.y, end_x, end_y);

	a_star.initialize(false, max_iterations, it_on_approach);
	a_star.setFootprint(footprint, false);
	a_star.createGraph(map->getSizeInCellsX(), map->getSizeInCellsY(), size_theta, map);
	a_star.setStart(start_x, start_y, 0u);
	a_star.setGoal(end_x, end_y, 0u);

	NodeSE2::CoordinateVector path;

	float tolerance = 5.0;
	int num_it = 0;
	bool found;
	double cost = 0.0;

	try
	{
		found = a_star.createPath(path, num_it, tolerance, cost);
	}
	catch (const std::runtime_error & e)
	{
		cerr << "failed to plan: " << e.what() << endl;
		return 1;
	}

	cout << "found " << found << endl;
	cout << "num_it " << num_it << " of max " << max_iterations << endl;
	cout << "path size " << path.size() << endl;
	cout << "cost " << cost << endl;

	if (!found)
		return 1;

	// Convert to world coordinates
	std::vector<Eigen::Vector2d> path_world;
	path_world.reserve(path.size());

	for (int i = path.size() - 1; i >= 0; --i)
	{
		double wx, wy;
		map->mapToWorld(path[i].x, path[i].y, wx, wy);
		path_world.push_back(Eigen::Vector2d(wx, wy));
	}

	Display(map, path_world);

	cv::waitKey(0);

	delete map;

	return 0;
}
