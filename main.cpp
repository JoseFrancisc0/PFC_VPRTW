#include <iostream>
#include <chrono>
#include <random>
#include <string>
#include "ALNS/alns.h"
#include "ALNS/alns_qlearning.h"
#include "ALNS/alns_dqn.h"
#include "Utils/utils.h"

unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
std::mt19937 rng(seed);

int test_benchmark() {
    try {
        std::cout << "==========================================\n";
        std::cout << "    ALNS - VEHICLE ROUTING PROBLEM (VRPTW)\n";
        std::cout << "==========================================\n";
        std::cout << "[INFO] Seed utilizada: " << seed << "\n";

        std::string instance_file = "solomon-100/rc2/rc201.txt";
        std::cout << "[1] Cargando instancia: " << instance_file << "...\n";
        Instance inst(instance_file);
        std::cout << "    -> Nodos cargados: " << inst.clients.size() << "\n";

        std::cout << "[2] Generando solucion inicial...\n";
        Solution initial_sol(inst);
        std::cout << initial_sol;

        int max_iterations = 25000;

        auto start_time = std::chrono::high_resolution_clock::now();
    
        // Elige uno
        Solution best_solution = solve_with_classic(inst, initial_sol, max_iterations, "Results/alns_metrics.csv");
        // Solution best_solution = solve_with_qlearning(inst, initial_sol, max_iterations, "Results/alns_qlearning_metrics.csv");
        // Solution best_solution = solve_with_dqn(inst, initial_sol, max_iterations, "Results/alns_dqn_metrics.csv", "");

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end_time - start_time;

        std::cout << "\n==========================================\n";
        std::cout << "             BUSQUEDA TERMINADA\n";
        std::cout << "==========================================\n";
        std::cout << best_solution;
        std::cout << "------------------------------------------\n";
        std::cout << "Tiempo de CPU: " << diff.count() << " segundos\n";

    } catch (const std::exception& e) {
        std::cerr << "ERROR FATAL: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

int minitest(const std::string& instance_file) {
    try {
        std::cout << "==========================================\n";
        std::cout << "    ALNS - VEHICLE ROUTING PROBLEM (VRPTW)\n";
        std::cout << "==========================================\n";

        std::cout << "[1] Cargando instancia: " << instance_file << "...\n";
        Instance inst(instance_file);
        std::cout << "    -> Nodos cargados: " << inst.clients.size() << "\n";

        std::cout << "[2] Generando solucion inicial...\n";
        Solution initial_sol(inst);
        std::cout << initial_sol;

        int max_iterations = 25000;

        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Elige uno
        Solution best_solution = solve_with_classic(inst, initial_sol, max_iterations, "");
        // Solution best_solution = solve_with_qlearning(inst, initial_sol, max_iterations, "");

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end_time - start_time;

        std::cout << "\n==========================================\n";
        std::cout << "             BUSQUEDA TERMINADA\n";
        std::cout << "==========================================\n";
        std::cout << best_solution;
        std::cout << "------------------------------------------\n";
        std::cout << "Tiempo de CPU: " << diff.count() << " segundos\n";

        // GENERANDO MEJOR SOLUCION EXACTA
        std::cout << "\n======MEJOR=SOLUCION=DE=LA=INSTANCIA======\n";

        Solution empty_sol(inst);
        empty_sol.routes.clear();

        int N = inst.clients.size();
        std::vector<bool> unassigned(N, true);
        unassigned[0] = false;
        int unassigned_count = N - 1;

        double best_cost = std::numeric_limits<double>::max();
        Solution best_sol(inst);

        std::cout << "Iniciando Fuerza Bruta (Branch and Bound)...\n";
        solveExact(empty_sol, unassigned, unassigned_count, best_cost, best_sol);

        std::cout << "\n=== OPTIMO GLOBAL ENCONTRADO ===\n";
        std::cout << best_sol;

        verifySolution(inst, best_sol);

    } catch (const std::exception& e) {
        std::cerr << "ERROR FATAL: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 1) {
        std::cout << "[MANUAL]\n";
        test_benchmark();
    }
    else if (argc >= 4) {
        try {
            std::string instance_file = argv[1];
            std::string algorithm = argv[2]; // "CLASSIC" / "QLEARNING" / "DQN"
            int max_iters = std::stoi(argv[3]);
            std::string run_id = (argc >= 5) ? argv[4] : "0";

            Instance inst(instance_file);
            Solution initial_sol(inst);

            size_t last_slash = instance_file.find_last_of("/\\");
            size_t last_dot = instance_file.find_last_of(".");
            std::string inst_name = instance_file.substr(last_slash + 1, last_dot - last_slash - 1);
            
            std::string metrics_file = "";
            std::string routes_file = "";
            std::string exp_file = "";

            if (run_id.find("NO_SAVE") == std::string::npos) {
                metrics_file = "../Results/" + algorithm + "/metrics/" + algorithm + "_" + inst_name + "_metrics_run" + run_id + ".csv";
                routes_file = "../Results/" + algorithm + "/routes/" + algorithm + "_" + inst_name + "_metrics_run" + run_id + ".csv";
                exp_file = "../DQN_Pipeline/experiences/experiences_" + inst_name + "_run" + run_id + ".csv";
            }

            Solution best_solution(inst);
            auto start_time = std::chrono::high_resolution_clock::now();

            if (algorithm == "CLASSIC")
                best_solution = solve_with_classic(inst, initial_sol, max_iters, metrics_file, routes_file, exp_file);
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
        } catch (const std::exception& e) {
            std::cerr << "ERROR FATAL: " << e.what() << "\n";
            return 1;
        }
    }
    else {
        std::cerr << "Uso incorrecto. Argumentos esperados: <instancia> <CLASSIC|QLEARNING|DQN> <iteraciones> [run_id]\n";
        return 1;
    }

    return 0;
}