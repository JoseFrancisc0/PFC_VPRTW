#ifndef ALNS_DQN_H
#define ALNS_DQN_H

#include <vector>
#include <functional>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random> 
#include <deque>
#include <torch/script.h> // LibTorch Header
#include "../Operators/operators.h"

struct IterationDataDQN {
    int iter;
    int best_vehicles;
    double best_distance;
    int curr_vehicles;
    double curr_distance;
    int d_idx;
    int r_idx;
    double temp;
};

using DestroyOp = std::function<void(Solution&, int)>;
using RepairOp  = std::function<void(Solution&)>;

class ALNS_DQN {
    public:
        ALNS_DQN(const Instance& _inst, const Solution& _initial_sol, const std::string& model_path);
        Solution solve(int max_iters, bool save_history = false);
        void exportMetrics(const std::string& filename);

    private:
        const Instance& inst;
        Solution current_sol;
        Solution best_sol;
        std::vector<IterationDataDQN> history;
        std::vector<DestroyOp> destroy_ops;
        std::vector<RepairOp> repair_ops;

        // Modelo de LibTorch
        torch::jit::script::Module expert_brain;
        bool model_loaded = false;

        // HIPERPARÁMETROS CALIBRADOS para SA (Simulated Annealing en Aceptación)
        double start_temp;
        double cooling_rate = 0.9998; 

        void initOps();
        bool accept(double cand_cost, double curr_cost, double current_temp);
};

Solution solve_with_dqn(const Instance& inst, const Solution& initial_sol, int max_iterations, const std::string& metrics_csv, const std::string& routes_csv);

#endif // ALNS_DQN_H
