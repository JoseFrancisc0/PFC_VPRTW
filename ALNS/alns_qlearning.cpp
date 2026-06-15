#include "alns_qlearning.h"
#include <iostream>
#include <fstream>

extern std::mt19937 rng; 

ALNS_QLearning::ALNS_QLearning(const Instance& _inst, const Solution& _initial_sol) 
    : inst(_inst), current_sol(_initial_sol), best_sol(_initial_sol) {
    initOps();
}

void ALNS_QLearning::initOps() {
    destroy_ops.push_back(randomRemoval);
    destroy_ops.push_back(routeRemoval);
    destroy_ops.push_back([](Solution& sol, int q) { worstRemoval(sol, q); });
    destroy_ops.push_back([](Solution& sol, int q) { shawRemoval(sol, q); });
    destroy_ops.push_back([](Solution& sol, int q) { timeWindowRemoval(sol, q); });

    repair_ops.push_back(greedyInsertion);
    repair_ops.push_back(regret2Insertion);
    repair_ops.push_back(regret3Insertion);
    repair_ops.push_back([](Solution& sol) { pGreedyInsertion(sol); });

    Q_table_D.assign(num_states, std::vector<double>(destroy_ops.size(), 0.0));
    Q_table_R.assign(num_states, std::vector<double>(repair_ops.size(), 0.0));
}

int ALNS_QLearning::selectOp(const std::vector<double>& q_values, double epsilon) {
    std::uniform_real_distribution<double> distr(0.0, 1.0);
    if (distr(rng) < epsilon) {
        std::uniform_int_distribution<int> act_distr(0, q_values.size() - 1);
        return act_distr(rng);
    } else {
        auto it = std::max_element(q_values.begin(), q_values.end());
        return std::distance(q_values.begin(), it);
    }
}

bool ALNS_QLearning::accept(double cand_cost, double curr_cost, double T) {
    double delta = cand_cost - curr_cost;
    if (delta <= 0) return true;
    double prob = std::exp(-delta / T);
    std::uniform_real_distribution<double> distr(0.0, 1.0);
    return distr(rng) < prob; 
}

Solution ALNS_QLearning::solve(int max_iters, bool save_metrics) {
    double initial_d = current_sol.total_distance;
    start_temp = -(0.10 * initial_d) / std::log(0.5);
    double T = start_temp;
    
    double epsilon = 1.0;
    double epsilon_decay = 0.998; 
    double epsilon_min = 0.15; 
    
    double eta = 0.8; 
    double C = 100.0; 
    double opportunity_cost = 0.0;

    int current_state = 0; 
    int n_customers = inst.clients.size() - 1;
    
    if (save_metrics) { history.reserve(max_iters); }

    for (int iter = 1; iter <= max_iters; ++iter) {
        Solution candidate = current_sol;

        int q_min = std::max(4, static_cast<int>(0.10 * n_customers));
        int q_max = std::max(q_min + 1, static_cast<int>(0.40 * n_customers));
        std::uniform_int_distribution<int> q_distr(q_min, q_max);
        int q = q_distr(rng);
        
        int d_idx = selectOp(Q_table_D[current_state], epsilon);
        int r_idx = selectOp(Q_table_R[current_state], epsilon);
        
        destroy_ops[d_idx](candidate, q);
        repair_ops[r_idx](candidate);
        
        double cand_cost = cost(candidate);
        double curr_cost = cost(current_sol);
        double best_cost = cost(best_sol);

        double delta_global = std::max((best_cost - cand_cost) / best_cost, 0.0);
        double delta_local  = std::max((curr_cost - cand_cost) / curr_cost, 0.0);
        double delta_improvement = (delta_global * eta) + (delta_local * (1.0 - eta));
        
        opportunity_cost = std::max(opportunity_cost, delta_improvement);

        double reward = 0.0;
        int next_state = 0;

        if (delta_improvement > 0) {
            reward = (delta_improvement * iter) / C;
            next_state = 1; 
        } else {
            reward = ((delta_improvement - opportunity_cost) * iter) / C;
            next_state = 0; 
        }

        if (cand_cost < best_cost) {
            best_sol = candidate;
            current_sol = candidate;
        } else if (cand_cost < curr_cost || accept(cand_cost, curr_cost, T)) {
            current_sol = candidate;
        }

        double max_next_q_D = *std::max_element(Q_table_D[next_state].begin(), Q_table_D[next_state].end());
        double max_next_q_R = *std::max_element(Q_table_R[next_state].begin(), Q_table_R[next_state].end());

        Q_table_D[current_state][d_idx] += alpha * (reward + gamma * max_next_q_D - Q_table_D[current_state][d_idx]);
        Q_table_R[current_state][r_idx] += alpha * (reward + gamma * max_next_q_R - Q_table_R[current_state][r_idx]);

        current_state = next_state;

        if (save_metrics) {
            IterationDataQL data;
            data.iter = iter;
            data.best_vehicles = best_sol.used_vehicles;
            data.best_distance = best_sol.total_distance;
            data.curr_vehicles = current_sol.used_vehicles;
            data.curr_distance = current_sol.total_distance;
            data.d_idx = d_idx;
            data.r_idx = r_idx;
            data.reward = reward; 
            data.temp = T;
            data.epsilon = epsilon;
            data.state = current_state;
            data.q_d_state0 = Q_table_D[0];
            data.q_d_state1 = Q_table_D[1];
            data.q_r_state0 = Q_table_R[0];
            data.q_r_state1 = Q_table_R[1];
            history.emplace_back(data);
        }
    
        T = T * cooling_rate; 
        epsilon = std::max(epsilon_min, epsilon * epsilon_decay); 
    }
    return best_sol;
}

void ALNS_QLearning::exportMetrics(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error al abrir archivo para metricas: " << filename << std::endl;
        return;
    }
    file << "iter,best_veh,best_dist,curr_veh,curr_dist,d_op,r_op,reward,temp,epsilon,state";
    for (size_t i = 0; i < destroy_ops.size(); ++i) file << ",q_d0_" << i;
    for (size_t i = 0; i < destroy_ops.size(); ++i) file << ",q_d1_" << i;
    for (size_t i = 0; i < repair_ops.size(); ++i) file << ",q_r0_" << i;
    for (size_t i = 0; i < repair_ops.size(); ++i) file << ",q_r1_" << i;
    file << "\n";
    
    for (const auto& data : history) {
        file << data.iter << ","
             << data.best_vehicles << ","
             << data.best_distance << ","
             << data.curr_vehicles << ","
             << data.curr_distance << ","
             << data.d_idx << ","
             << data.r_idx << ","
             << data.reward << ","
             << data.temp << ","
             << data.epsilon << ","
             << data.state << ",";
        for (double q : data.q_d_state0) file << "," << q;
        for (double q : data.q_d_state1) file << "," << q;
        for (double q : data.q_r_state0) file << "," << q;
        for (double q : data.q_r_state1) file << "," << q;
        file << "\n";
    }

    file.close();
    std::cout << "-> Metricas exportadas a " << filename << "\n";
}