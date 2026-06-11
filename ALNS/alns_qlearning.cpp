#include "alns_qlearning.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>

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

    num_states = 6; 
    // Tablas separadas para Destrucción y Reparación
    Q_table_D.assign(num_states, std::vector<double>(destroy_ops.size(), 10.0));
    Q_table_R.assign(num_states, std::vector<double>(repair_ops.size(), 10.0));
}

int ALNS_QLearning::getState(int iters_without_improvement, const Solution& sol) const {
    int stag_level = 0;
    if (iters_without_improvement > 300) stag_level = 2;
    else if (iters_without_improvement > 100) stag_level = 1;

    double total_load = 0.0;
    int num_routes = 0;
    for (const auto& r : sol.routes) {
        if (r.path.size() > 2) {
            total_load += r.load;
            num_routes++;
        }
    }
    double capacity_util = (num_routes > 0) ? (total_load / (num_routes * inst.capacity)) : 0.0;
    int cap_level = (capacity_util >= 0.75) ? 1 : 0;

    return stag_level * 2 + cap_level;
}

int ALNS_QLearning::selectOp(const std::vector<double>& q_values, double tau) {
    double max_q = *std::max_element(q_values.begin(), q_values.end());
    std::vector<double> probs(q_values.size());
    for (size_t i = 0; i < q_values.size(); ++i) {
        probs[i] = std::exp((q_values[i] - max_q) / tau);
    }
    std::discrete_distribution<int> dist(probs.begin(), probs.end());
    return dist(rng);
}

bool ALNS_QLearning::accept(double cand_cost, double curr_cost, double T) {
    double delta = cand_cost - curr_cost;
    if (delta <= 0) return true;
    double prob = std::exp(-delta / T);
    std::uniform_real_distribution<double> distr(0.0, 1.0);
    return distr(rng) < prob; 
}

Solution ALNS_QLearning::solve(int max_iters, bool save_history) {
    double initial_d = current_sol.total_distance;
    // Ajuste: Temperatura inicial más suave para no destruir buenas rutas iniciales
    start_temp = -(0.05 * initial_d) / std::log(0.5);
    double T = start_temp;
    
    // Softmax temperature
    double tau = 5.0;  
    double tau_min = 0.1;  
    double tau_decay = 0.9995;

    struct ActionRecord { int state; int d_action; int r_action; };
    std::deque<ActionRecord> action_buffer;

    int n_customers = inst.clients.size() - 1;
    history.reserve(max_iters);

    int iters_without_improvement = 0;
    int current_state = getState(iters_without_improvement, current_sol); 

    for (int iter = 0; iter < max_iters; ++iter) {
        Solution candidate = current_sol;
        
        int stag_level = current_state / 2;
        int q_min, q_max;
        if (stag_level == 0) { 
            q_min = std::max(4, static_cast<int>(0.10 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.15 * n_customers));
        } else if (stag_level == 1) { 
            q_min = std::max(4, static_cast<int>(0.15 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.20 * n_customers));
        } else { 
            q_min = std::max(4, static_cast<int>(0.20 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.25 * n_customers));
        }
        std::uniform_int_distribution<int> q_distr(q_min, q_max);
        int q = q_distr(rng);
        
        int d_idx = selectOp(Q_table_D[current_state], tau);
        int r_idx = selectOp(Q_table_R[current_state], tau);
        
        destroy_ops[d_idx](candidate, q);
        repair_ops[r_idx](candidate);
        
        double cand_cost = cost(candidate);
        double curr_cost = cost(current_sol);
        double best_cost = cost(best_sol);

        double reward = 0.0;
        if (candidate.used_vehicles < current_sol.used_vehicles) {
            reward = 10.0;
        } else if (candidate.used_vehicles > current_sol.used_vehicles) {
            reward = -10.0;
        } else {
            reward = ((current_sol.total_distance - candidate.total_distance) / current_sol.total_distance) * 100.0;
        }

        if (cand_cost < best_cost) {
            best_sol = candidate;
            current_sol = candidate;
            iters_without_improvement = 0;
        }
        else if (cand_cost < curr_cost) {
            current_sol = candidate;
            iters_without_improvement = 0;
        }
        else if (accept(cand_cost, curr_cost, T)) { 
            current_sol = candidate;
            iters_without_improvement++;
        } else {
            iters_without_improvement++;
            reward -= 2.0; // Penalización base por solución rechazada
        }
        
        reward = std::max(-10.0, std::min(10.0, reward)); // Limitar para estabilidad

        int next_state = getState(iters_without_improvement, current_sol); 
        if (iters_without_improvement > 1500) {
            T = start_temp; 
            tau = 5.0; // Resetear exploracion
            iters_without_improvement = 0;
            next_state = getState(0, current_sol);
        }

        // N-Step Reward Buffer Update
        action_buffer.push_back({current_state, d_idx, r_idx});
        if(action_buffer.size() > 5) action_buffer.pop_front();

        double max_next_q_D = *std::max_element(Q_table_D[next_state].begin(), Q_table_D[next_state].end());
        double max_next_q_R = *std::max_element(Q_table_R[next_state].begin(), Q_table_R[next_state].end());

        for (size_t i = 0; i < action_buffer.size(); ++i) {
            int s = action_buffer[i].state;
            int d_a = action_buffer[i].d_action;
            int r_a = action_buffer[i].r_action;
            double discount = std::pow(gamma, action_buffer.size() - 1 - i);
            Q_table_D[s][d_a] += alpha * ((reward * discount) + gamma * max_next_q_D - Q_table_D[s][d_a]);
            Q_table_R[s][r_a] += alpha * ((reward * discount) + gamma * max_next_q_R - Q_table_R[s][r_a]);
        }

        current_state = next_state;
        
        if (save_history) {
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
            data.epsilon = tau; 
            history.emplace_back(data);
        }
        
        T = T * cooling_rate; 
        tau = std::max(tau_min, tau * tau_decay); 
    }
    return best_sol;
}

void ALNS_QLearning::exportMetrics(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error al abrir archivo para metricas: " << filename << std::endl;
        return;
    }
    file << "iter,best_veh,best_dist,curr_veh,curr_dist,d_op,r_op,reward,temp,epsilon\n";
    
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
             << data.epsilon << "\n";
    }
    file.close();
    std::cout << "-> Metricas exportadas a " << filename << "\n";
}