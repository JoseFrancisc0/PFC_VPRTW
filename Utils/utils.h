#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <string>
#include "../ALNS/alns.h"
#include "../ALNS/alns_qlearning.h"

bool verifySolution(const Instance& inst, const Solution& sol);
void solveExact(Solution& current_sol, std::vector<bool>& unassigned, int unassigned_count, double& best_cost, Solution& best_sol);
void exportSolutionRoutes(const Solution& sol, const std::string& filename);

Solution solve_with_classic(const Instance& inst, const Solution& sol, int max_iters, std::string metrics_path = "", std::string routes_path = "");
Solution solve_with_qlearning(const Instance& inst, const Solution& sol, int max_iters, std::string metrics_path = "", std::string routes_path = "");

#endif //UTILS_H