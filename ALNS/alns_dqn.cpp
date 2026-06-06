#include "alns_dqn.h"
#include "../Utils/utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>

extern std::mt19937 rng; // Utilizar el mismo rng definido en main.cpp

ALNS_DQN::ALNS_DQN(const Instance& _inst, const Solution& _initial_sol, const std::string& model_path) 
    : inst(_inst), current_sol(_initial_sol), best_sol(_initial_sol) {
    initOps();
    try {
        expert_brain = torch::jit::load(model_path);
        model_loaded = true;
        std::cout << "[DQN] Modelo cargado exitosamente desde: " << model_path << "\n";
    } catch (const c10::Error& e) {
        std::cerr << "[DQN] Error al cargar el modelo " << model_path << "\n";
        model_loaded = false;
    }
}

void ALNS_DQN::initOps() {
    destroy_ops.push_back(randomRemoval);
    destroy_ops.push_back(routeRemoval);
    destroy_ops.push_back([](Solution& sol, int q) { worstRemoval(sol, q); });
    destroy_ops.push_back([](Solution& sol, int q) { shawRemoval(sol, q); });
    destroy_ops.push_back([](Solution& sol, int q) { timeWindowRemoval(sol, q); });

    repair_ops.push_back(greedyInsertion);
    repair_ops.push_back(regret2Insertion);
    repair_ops.push_back(regret3Insertion);
    repair_ops.push_back([](Solution& sol) { pGreedyInsertion(sol); });
}

bool ALNS_DQN::accept(double cand_cost, double curr_cost, double T) {
    double delta = cand_cost - curr_cost;
    if (delta <= 0) return true;
    double prob = std::exp(-delta / T);
    std::uniform_real_distribution<double> distr(0.0, 1.0);
    return distr(rng) < prob; 
}

Solution ALNS_DQN::solve(int max_iters) {
    double initial_d = current_sol.total_distance;
    start_temp = -(0.05 * initial_d) / std::log(0.5);
    double T = start_temp;
    
    int n_customers = inst.clients.size() - 1;
    history.reserve(max_iters);

    int iters_without_improvement = 0;

    for (int iter = 0; iter < max_iters; ++iter) {
        Solution candidate = current_sol;
        
        // Determinar magnitud de destruccion basado en estancamiento
        int q_min, q_max;
        if (iters_without_improvement < 100) { 
            q_min = std::max(4, static_cast<int>(0.10 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.15 * n_customers));
        } else if (iters_without_improvement < 300) { 
            q_min = std::max(4, static_cast<int>(0.15 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.20 * n_customers));
        } else { 
            q_min = std::max(4, static_cast<int>(0.20 * n_customers));
            q_max = std::max(q_min + 1, static_cast<int>(0.25 * n_customers));
        }
        std::uniform_int_distribution<int> q_distr(q_min, q_max);
        int q = q_distr(rng);
        
        // 1. Extraer estado
        std::vector<float> state_features = current_sol.extract_state_features(iters_without_improvement, iter, max_iters);
        
        int d_idx = 0;
        int r_idx = 0;
        int action = 0;

        if(model_loaded) {
            // 2. Inferencia DQN
            torch::Tensor state_tensor = torch::from_blob(state_features.data(), {1, 105}).clone();
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(state_tensor);

            torch::Tensor q_values = expert_brain.forward(inputs).toTensor();
            
            // Opcional: Epsilon-Greedy minimo para salir de loops infinitos (e.g., 5% exploracion)
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if(dist(rng) < 0.05) {
                std::uniform_int_distribution<int> rand_action(0, (destroy_ops.size() * repair_ops.size()) - 1);
                action = rand_action(rng);
            } else {
                action = q_values.argmax(1).item<int>();
            }
            
            d_idx = action / repair_ops.size();
            r_idx = action % repair_ops.size();
        } else {
            // Fallback: Random si no hay modelo
            std::uniform_int_distribution<int> rand_d(0, destroy_ops.size() - 1);
            std::uniform_int_distribution<int> rand_r(0, repair_ops.size() - 1);
            d_idx = rand_d(rng);
            r_idx = rand_r(rng);
        }
        
        destroy_ops[d_idx](candidate, q);
        repair_ops[r_idx](candidate);
        
        double cand_cost = cost(candidate);
        double curr_cost = cost(current_sol);
        double best_cost = cost(best_sol);

        bool vehicle_reduced = candidate.used_vehicles < best_sol.used_vehicles;
        bool distance_improved = (candidate.used_vehicles == best_sol.used_vehicles && cand_cost < best_cost);

        if (vehicle_reduced || distance_improved) {
            best_sol = candidate;
            current_sol = candidate;
            iters_without_improvement = 0;
        }
        else if (candidate.used_vehicles < current_sol.used_vehicles || 
                (candidate.used_vehicles == current_sol.used_vehicles && cand_cost < curr_cost)) {
            current_sol = candidate;
            iters_without_improvement = 0;
        }
        else if (cand_cost == best_cost && candidate.used_vehicles == best_sol.used_vehicles) {
            current_sol = candidate;
            iters_without_improvement++; 
        }
        else if (accept(cand_cost, curr_cost, T)) { 
            current_sol = candidate;
            iters_without_improvement++;
        } else {
            iters_without_improvement++;
        }

        // Reheating (SA)
        if (iters_without_improvement > 1500) {
            T = start_temp; 
            iters_without_improvement = 0;
        }

        IterationDataDQN data;
        data.iter = iter;
        data.best_vehicles = best_sol.used_vehicles;
        data.best_distance = best_sol.total_distance;
        data.curr_vehicles = current_sol.used_vehicles;
        data.curr_distance = current_sol.total_distance;
        data.d_idx = d_idx;
        data.r_idx = r_idx;
        data.temp = T;
        history.emplace_back(data);
        
        T = T * cooling_rate; 
    }
    return best_sol;
}

void ALNS_DQN::exportMetrics(const std::string& filename) {
    if(filename.empty()) return;
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error al abrir archivo para metricas: " << filename << std::endl;
        return;
    }
    file << "iter,best_veh,best_dist,curr_veh,curr_dist,d_op,r_op,temp\n";
    for (const auto& data : history) {
        file << data.iter << ","
             << data.best_vehicles << ","
             << data.best_distance << ","
             << data.curr_vehicles << ","
             << data.curr_distance << ","
             << data.d_idx << ","
             << data.r_idx << ","
             << data.temp << "\n";
    }
    file.close();
    std::cout << "-> Metricas exportadas a " << filename << "\n";
}

Solution solve_with_dqn(const Instance& inst, const Solution& initial_sol, int max_iterations, const std::string& metrics_csv, const std::string& routes_csv) {
    std::cout << "[3] Iniciando ALNS con DQN por " << max_iterations << " iteraciones...\n";
    ALNS_DQN dqn(inst, initial_sol, "DQN_Pipeline/alns_dqn_expert.pt");
    Solution best = dqn.solve(max_iterations);
    
    if (!metrics_csv.empty()) dqn.exportMetrics(metrics_csv);
    if (!routes_csv.empty()) exportSolutionRoutes(best, routes_csv);
    
    verifySolution(inst, best);
    return best;
}
