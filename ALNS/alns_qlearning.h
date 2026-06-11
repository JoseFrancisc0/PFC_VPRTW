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

// Estructura para almacenar las métricas de cada iteración
struct IterationDataQL {
    int iter;
    int best_vehicles;
    double best_distance;
    int curr_vehicles;
    double curr_distance;
    int d_idx;       // Índice del operador de destrucción
    int r_idx;       // Índice del operador de reparación
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
        
        // Listas de operadores independientes
        std::vector<DestroyOp> destroy_ops;
        std::vector<RepairOp> repair_ops;

        // HIPERPARÁMETROS
        double start_temp;
        double cooling_rate = 0.9998; 
        
        double alpha = 0.05; 
        double gamma = 0.8; 
        
        // Estado binario: 0 (Sin mejora), 1 (Con mejora)
        int num_states = 2; 
        
        // Tablas Q independientes
        std::vector<std::vector<double>> Q_table_D;
        std::vector<std::vector<double>> Q_table_R;

        void initOps();
        bool accept(double cand_cost, double curr_cost, double current_temp);
        int selectOp(const std::vector<double>& q_values, double epsilon);
};

#endif