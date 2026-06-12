#include <iostream>
#include <chrono>
#include <string>
#include "ALNS/alns.h"
#include "ALNS/alns_qlearning.h"
#include "ALNS/alns_dqn.h"
#include "Utils/utils.h"

unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
std::mt19937 rng(seed);

void manual_run() {
    std::cout << "==========================================\n";
    std::cout << "        EJECUCION MANUAL (NO SAVE)\n";
    std::cout << "==========================================\n";
    
    std::string instance_file = "solomon-100/rc1/rc101.txt";
    std::string algorithm = "QLEARNING";
    int max_iters = 25000;

    std::cout << "[INFO] Instancia: " << instance_file << "\n";
    std::cout << "[INFO] Algoritmo: " << algorithm << "\n";
    std::cout << "[INFO] Iteraciones: " << max_iters << "\n";

    Instance inst(instance_file);
    Solution initial_sol(inst);
    Solution best_solution(inst);

    auto start_time = std::chrono::high_resolution_clock::now();

    if (algorithm == "CLASSIC")
        best_solution = solve_with_classic(inst, initial_sol, max_iters, "", "", "", false);
    else if (algorithm == "QLEARNING")
        best_solution = solve_with_qlearning(inst, initial_sol, max_iters, "", "");
    else if (algorithm == "DQN")
        best_solution = solve_with_dqn(inst, initial_sol, max_iters, "", "");

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;

    std::cout << "\n==========================================\n";
    std::cout << "             BUSQUEDA TERMINADA\n";
    std::cout << "==========================================\n";
    std::cout << best_solution;
    std::cout << "------------------------------------------\n";
    std::cout << "Tiempo de CPU: " << diff.count() << " segundos\n";
}

int main(int argc, char** argv) {
    if (argc == 1) {
        manual_run();
        return 0;
    }

    if (argc < 4) {
        std::cerr << "Uso: " << argv[0] << " <ruta_instancia> <ALGORITMO> <max_iters> [modo_ejecucion]\n";
        return 1;
    }
    
    try {
        std::string instance_file = argv[1];
        std::string algorithm = argv[2]; 
        int max_iters = std::stoi(argv[3]);
        std::string exec_mode = (argc >= 5) ? argv[4] : "MANUAL_SAVE";
        
        size_t last_slash = instance_file.find_last_of("/\\");
        size_t last_dot = instance_file.find_last_of(".");
        std::string inst_name = instance_file.substr(last_slash + 1, last_dot - last_slash - 1);

        bool generate_experiences = false;
        std::string metrics_file = "";
        std::string routes_file = "";
        std::string exp_file = "";

        if (exec_mode == "GENERATE_DATA") {
            generate_experiences = true;
            exp_file = "../DQN_Pipeline/experiences/experiences_" + inst_name + "_dataset.csv";
        }
        else if (exec_mode == "BENCHMARK") {
            // Ni se generan experiencias, ni se guarda history metrics a disco.
        }
        else if (exec_mode == "SAVE_HISTORY") {
            metrics_file = "../Results/" + algorithm + "/metrics/" + algorithm + "_" + inst_name + "_metrics.csv";
            routes_file = "../Results/" + algorithm + "/routes/" + algorithm + "_" + inst_name + "_routes.csv";
        }
        else {
            if (exec_mode.find("NO_SAVE") == std::string::npos) {
                metrics_file = "../Results/" + algorithm + "/metrics/" + algorithm + "_" + inst_name + "_metrics_run_" + exec_mode + ".csv";
                routes_file = "../Results/" + algorithm + "/routes/" + algorithm + "_" + inst_name + "_routes_run_" + exec_mode + ".csv";
            }
        }

        Instance inst(instance_file);
        Solution initial_sol(inst);
        Solution best_solution(inst);
        auto start_time = std::chrono::high_resolution_clock::now();

        if (algorithm == "CLASSIC")
            best_solution = solve_with_classic(inst, initial_sol, max_iters, metrics_file, routes_file, exp_file, generate_experiences);
        else if (algorithm == "QLEARNING")
            best_solution = solve_with_qlearning(inst, initial_sol, max_iters, metrics_file, routes_file);
        else if (algorithm == "DQN")
            best_solution = solve_with_dqn(inst, initial_sol, max_iters, metrics_file, routes_file);
        else {
            std::cerr << "Algoritmo desconocido: " << algorithm << "\n";
            return 1;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end_time - start_time;

        std::cout << "\n==========================================\n";
        std::cout << "             BUSQUEDA TERMINADA\n";
        std::cout << "==========================================\n";
        std::cout << best_solution;
        std::cout << "------------------------------------------\n";
        std::cout << "Tiempo de CPU: " << diff.count() << " segundos\n";
        std::cout << "[FINAL_RESULT] Veh: " << best_solution.used_vehicles << ", Dist: " << best_solution.total_distance << "\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR FATAL: " << e.what() << "\n";
        return 1;
    }

    return 0;
}