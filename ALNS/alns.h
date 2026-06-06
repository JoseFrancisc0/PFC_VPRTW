#ifndef ALNS_H
#define ALNS_H

#include <vector>
#include <functional>
#include <string>
#include "../Operators/operators.h"

// Para guardar metricas de analisis
struct IterationData {
    int iter;
    int best_vehicles;
    double best_distance;
    int curr_vehicles;
    double curr_distance;
    int d_idx;
    int r_idx;
    double score;
    double temp;

    std::vector<double> d_weights;
    std::vector<double> r_weights;
};

struct ExperienceRecord {
    std::vector<float> state;
    int action;
    double reward;
    std::vector<float> next_state;
};

using DestroyOp = std::function<void(Solution&, int)>;
using RepairOp  = std::function<void(Solution&)>;

class ALNS {
    public:
        ALNS(const Instance& _inst, const Solution& _initial_sol);
        Solution solve(int max_iters);
        void exportMetrics(const std::string& filename);
        void exportExperiences(const std::string& filename);

    private:
        const Instance& inst;
        Solution current_sol;
        Solution best_sol;

        std::vector<IterationData> history; // para guardar metricas
        std::vector<ExperienceRecord> experiences; // para Dataset DQN

        // Operadores de destroy (Omega^-)
        std::vector<DestroyOp> destroy_ops;
        std::vector<double> destroy_weights;

        // Operadores de repair (Omega^+)
        std::vector<RepairOp> repair_ops;
        std::vector<double> repair_weights;

        // Hiperparametros del ALNS
        double decay = 0.9;
        double w1 = 33.0;
        double w2 = 13.0;
        double w3 = 9.0;
        double w4 = 0.0;

        // Hiperparametros del Simulated Annealing
        double start_temp;
        double cooling_rate = 0.9995;

        void initOps();
        int selectDestroyOp();
        int selectRepairOp();
        bool accept(double cand_cost, double curr_cost, double current_temp);
        void updateWeights(int used_destroy_idx, int used_repair_idx, double score);
};

#endif //ALNS_H