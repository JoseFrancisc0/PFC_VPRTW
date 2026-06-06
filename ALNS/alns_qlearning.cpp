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

    num_states = 3; 
    // Matriz única combinando todas las posibles parejas de (destroy_op, repair_op)
    Q_table.assign(num_states, std::vector<double>(destroy_ops.size() * repair_ops.size(), 10.0));
}

int ALNS_QLearning::selectPair(const std::vector<double>& q_values, double epsilon) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < epsilon) {
        std::uniform_int_distribution<int> rand_action(0, q_values.size() - 1);
        return rand_action(rng);
    }
    int best_a = 0;
    double max_val = q_values[0];
    for(size_t i = 1; i < q_values.size(); i++){
        if(q_values[i] > max_val){
            max_val = q_values[i];
            best_a = i;
        }
    }
    return best_a;
}

bool ALNS_QLearning::accept(double cand_cost, double curr_cost, double T) {
    double delta = cand_cost - curr_cost;
    if (delta <= 0) return true;
    double prob = std::exp(-delta / T);
    std::uniform_real_distribution<double> distr(0.0, 1.0);
    return distr(rng) < prob; 
}

Solution ALNS_QLearning::solve(int max_iters) {
    double initial_d = current_sol.total_distance;
    // Ajuste: Temperatura inicial más suave para no destruir buenas rutas iniciales
    start_temp = -(0.05 * initial_d) / std::log(0.5);
    double T = start_temp;
    
    // Ajuste E-Greedy
    double epsilon = 0.5;  
    double epsilon_min = 0.05;  
    double epsilon_decay = 0.9995;

    struct ActionRecord { int state; int action; };
    std::deque<ActionRecord> action_buffer;

    int n_customers = inst.clients.size() - 1;
    history.reserve(max_iters);

    int current_state = 0; 
    int iters_without_improvement = 0;

    for (int iter = 0; iter < max_iters; ++iter) {
        Solution candidate = current_sol;
        
        int q_min, q_max;
        if (current_state == 0) { 
            q_min = std::max(4, static_cast<int>(0.10 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.15 * n_customers));
        } else if (current_state == 1) { 
            q_min = std::max(4, static_cast<int>(0.15 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.20 * n_customers));
        } else { 
            q_min = std::max(4, static_cast<int>(0.20 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.25 * n_customers));
        }
        std::uniform_int_distribution<int> q_distr(q_min, q_max);
        int q = q_distr(rng);
        
        int action = selectPair(Q_table[current_state], epsilon);
        int d_idx = action / repair_ops.size();
        int r_idx = action % repair_ops.size();
        
        destroy_ops[d_idx](candidate, q);
        repair_ops[r_idx](candidate);
        
        double reward = w4;
        bool global_improved = false;
        
        double cand_cost = cost(candidate);
        double curr_cost = cost(current_sol);
        double best_cost = cost(best_sol);

        // EVALUACIÓN JERÁRQUICA: 1. Minimizar Vehículos, 2. Minimizar Distancia
        bool vehicle_reduced = candidate.used_vehicles < best_sol.used_vehicles;
        bool distance_improved = (candidate.used_vehicles == best_sol.used_vehicles && cand_cost < best_cost);

        if (vehicle_reduced || distance_improved) {
            best_sol = candidate;
            current_sol = candidate;
            // Doble recompensa si logra reducir un vehículo
            reward = vehicle_reduced ? w1 * 2.0 : w1; 
            global_improved = true;
            iters_without_improvement = 0;
        }
        else if (candidate.used_vehicles < current_sol.used_vehicles || 
                (candidate.used_vehicles == current_sol.used_vehicles && cand_cost < curr_cost)) {
            current_sol = candidate;
            reward = w2; 
            iters_without_improvement = 0;
        }
        else if (cand_cost == best_cost && candidate.used_vehicles == best_sol.used_vehicles) {
            current_sol = candidate;
            reward = w2;
            iters_without_improvement++; 
        }
        else if (accept(cand_cost, curr_cost, T)) { 
            if(candidate.used_vehicles <= current_sol.used_vehicles) {
                current_sol = candidate;
                reward = w3; 
            } else {
                // Penalización severa por empeorar vehículos y ser aceptada probabilísticamente
                reward = -5.0; 
            }
            iters_without_improvement++;
        } else {
            if(candidate.used_vehicles > current_sol.used_vehicles) {
                // Penalización moderada por proponer una mala ruta y ser rechazada
                reward = -2.0;
            }
            iters_without_improvement++;
        }

        // Reheating adaptado a epsilon
        int next_state = 0; 
        if (iters_without_improvement > 300) {
            next_state = 2; 
            if (iters_without_improvement > 1500) {
                T = start_temp; 
                epsilon = 0.5; // Resetear exploracion
                iters_without_improvement = 0;
            }
        } else if (iters_without_improvement > 100) {
            next_state = 1; 
        }

        // N-Step Reward Buffer Update
        action_buffer.push_back({current_state, action});
        if(action_buffer.size() > 5) action_buffer.pop_front();

        double max_next_q = *std::max_element(Q_table[next_state].begin(), Q_table[next_state].end());
        for (size_t i = 0; i < action_buffer.size(); ++i) {
            int s = action_buffer[i].state;
            int a = action_buffer[i].action;
            double discount = std::pow(gamma, action_buffer.size() - 1 - i);
            Q_table[s][a] += alpha * ((reward * discount) + gamma * max_next_q - Q_table[s][a]);
        }

        current_state = next_state;
        
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
        history.emplace_back(data);
        
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