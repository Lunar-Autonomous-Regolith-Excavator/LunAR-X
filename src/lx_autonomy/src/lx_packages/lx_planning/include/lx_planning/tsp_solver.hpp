// Copyright 2010-2022 Google LLC
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

#ifndef TSP_SOLVER_HPP
#define TSP_SOLVER_HPP

#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace operations_research;
class GoogleTSPSolver {
public:

    //! @brief Print the solution.
    //! @param[in] manager Index manager used.
    //! @param[in] routing Routing solver used.
    //! @param[in] solution Solution found by the solver.
    void PrintSolution(const RoutingIndexManager& manager,
                    const RoutingModel& routing, const Assignment& solution) {
        // Inspect solution.
        LOG(INFO) << "Objective: " << solution.ObjectiveValue() << " miles";
        int64_t index = routing.Start(0);
        LOG(INFO) << "Route:";
        int64_t distance{0};
        std::stringstream route;
        while (!routing.IsEnd(index)) {
            route << manager.IndexToNode(index).value() << " -> ";
            const int64_t previous_index = index;
            index = solution.Value(routing.NextVar(index));
            distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{0});
        }
        LOG(INFO) << route.str() << manager.IndexToNode(index).value();
        LOG(INFO) << "Route distance: " << distance << "miles";
        LOG(INFO) << "";
        LOG(INFO) << "Advanced usage:";
        LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
    }

    int solve(const std::vector<std::vector<int64_t>> distance_matrix, int start_node_idx = 0, bool verbose = false) {
        RoutingIndexManager manager(distance_matrix.size(), 1, RoutingIndexManager::NodeIndex({start_node_idx}));
        RoutingModel routing(manager);

        const int transit_callback_index = routing.RegisterTransitCallback(
            [&distance_matrix, &manager](const int64_t from_index,
                                        const int64_t to_index) -> int64_t {
            // Convert from routing variable Index to distance matrix NodeIndex.
            const int from_node = manager.IndexToNode(from_index).value();
            const int to_node = manager.IndexToNode(to_index).value();
            return distance_matrix[from_node][to_node];
            });

        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
        RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

        const Assignment* solution = routing.SolveWithParameters(searchParameters);
        if (verbose)
            PrintSolution(manager, routing, *solution);

        return solution->ObjectiveValue();
    }

};

#endif // TSP_SOLVER_HPP