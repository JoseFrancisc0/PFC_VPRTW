#ifndef ALNS_QLEARNING_H
#define ALNS_QLEARNING_H

#include <vector>
#include <functional>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random> 
#include <deque>
#include "../Operators/operators.h"

struct IterationDataQL {
    int iter;
    int best_vehicles;
    double best_distance;
    int curr_vehicles;
    double curr_distance;
    int d_idx;
    int r_idx;
    double reward;
    double temp;
    double epsilon;
};

using DestroyOp = std::function<void(Solution&, int)>;
using RepairOp  = std::function<void(Solution&)>;

class ALNS_QLearning {
    public:
        ALNS_QLearning(const Instance& _inst, const Solution& _initial_sol);
        Solution solve(int max_iters, bool save_history = false);
        void exportMetrics(const std::string& filename);

    private:
        const Instance& inst;
        Solution current_sol;
        Solution best_sol;
        std::vector<IterationDataQL> history;
        std::vector<DestroyOp> destroy_ops;
        std::vector<RepairOp> repair_ops;

        // HIPERPARÁMETROS CALIBRADOS
        double start_temp;
        double cooling_rate = 0.9998; // Enfriamiento simulado estándar
        
        double alpha = 0.05; 
        double gamma = 0.8; 
        int num_states = 6;
        std::vector<std::vector<double>> Q_table_D;
        std::vector<std::vector<double>> Q_table_R;

        void initOps();
        bool accept(double cand_cost, double curr_cost, double current_temp);
        int selectOp(const std::vector<double>& q_values, double tau);
        int getState(int iters_without_improvement, const Solution& sol) const;
};

#endif