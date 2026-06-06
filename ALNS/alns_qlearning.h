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
        Solution solve(int max_iters);
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
        
        // Recompensas acotadas para no saturar la función Softmax
        double w1 = 10.0;  // Óptimo global
        double w2 = 5.0;   // Mejora local
        double w3 = 2.0;   // Solución aceptada
        double w4 = 0.0;   // Solución rechazada
        
        double alpha = 0.05; 
        double gamma = 0.8; 
        int num_states = 3;
        std::vector<std::vector<double>> Q_table;

        void initOps();
        bool accept(double cand_cost, double curr_cost, double current_temp);
        int selectPair(const std::vector<double>& q_values, double epsilon);
};

#endif