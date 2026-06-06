#include "solution.h"

// Para imprimir soluciones
std::ostream& operator<<(std::ostream& os, const Solution& s){
    os << "Vehiculos usados: " << s.used_vehicles << "  - Distancia total: " << s.total_distance << "\n";

    int r = 0;
    for (const Route& route : s.routes) {
        if (route.path.size() > 2) {
            os << "Ruta [" << r << "]: ";
            for (const int& i : route.path) {
                if (i != 0)
                    os << i << " ";
            }
            os << "\n";
            r++;
        }
    }

    return os;
}

// Comparador de soluciones (jerarquia de objetivos)
bool Solution::operator<(const Solution& other) const {
    if (this->used_vehicles < other.used_vehicles) return true;
    if (this->used_vehicles > other.used_vehicles) return false;

    return this->total_distance < other.total_distance;
}

// Operador copia para soluciones
Solution& Solution::operator=(const Solution& other) {
    if (this == &other) return *this;

    this->routes = other.routes;
    this->unassigned = other.unassigned;
    this->total_distance = other.total_distance;
    this->used_vehicles = other.used_vehicles;

    return *this;
}

// Actualizar metricas objetivo
void Solution::updateMetrics() {
    used_vehicles = 0;
    total_distance = 0.0;

    for (const Route& route: routes) {
        if (route.path.size() > 2) {
            used_vehicles++;
            total_distance += route.distance;
        }
    }
}

// Generar solucion inicial (NN-Based)
void Solution::generateInitialSolution() {
    int N = inst.clients.size();
    std::vector<bool> visited(N, false);
    visited[0] = true;
    int unvisited = N - 1;

    while (unvisited > 0) {
        Route current_route;
        int current_node = 0;
        double current_time = 0.0;
        double current_load = 0.0;

        bool added_client = true;

        while (added_client) {
            added_client = false;
            int best_client = -1;
            double best_distance = std::numeric_limits<double>::max();

            for (int i = 1; i < N; ++i) {
                if (visited[i]) continue;
                if (!inst.is_reachable[current_node][i]) continue;
                if (current_load + inst.clients[i].demand > inst.capacity) continue;

                double arrival_time = current_time + inst.clients[current_node].service_time + inst.dist_mat[current_node][i];
                arrival_time = std::max(arrival_time, inst.clients[i].ready_time);

                if (arrival_time <= inst.clients[i].due_date) {
                    double return_time = arrival_time + inst.clients[i].service_time + inst.dist_mat[i][0];

                    if (return_time <= inst.clients[0].due_date) {
                        if (inst.dist_mat[current_node][i] < best_distance) {
                            best_distance = inst.dist_mat[current_node][i];
                            best_client = i;
                        }
                    }
                }
            }

            if (best_client != -1) {
                current_route.path.insert(current_route.path.end() - 1, best_client);
                visited[best_client] = true;
                unvisited--;
                
                current_load += inst.clients[best_client].demand;
                double arrival = current_time + inst.clients[current_node].service_time + inst.dist_mat[current_node][best_client];
                current_time = std::max(arrival, inst.clients[best_client].ready_time);
                
                current_route.distance += best_distance;
                current_route.load = current_load;
                
                current_node = best_client;
                added_client = true;
            }
            else {
                if (current_node == 0) {
                    for (int i = 1; i < N; ++i) {
                        if (!visited[i]) {
                            unassigned.push_back(i);
                            visited[i] = true;
                            unvisited--;
                        }
                    }
                    break;
                }
            }
        }
        
        current_route.distance += inst.dist_mat[current_node][0];
        current_route.recalculate(inst);
        routes.push_back(current_route);
    }

    updateMetrics();
}

// Calcular costos de solucion
double cost(const Solution& sol) {
    const double VEHICLE_COST = 50000.0;
    return (sol.used_vehicles * VEHICLE_COST) + sol.total_distance;
}

// Extraer vector de estado (Generalizado) para DQN
std::vector<float> Solution::extract_state_features(int iters_without_improvement, int current_iter, int max_iters) const {
    std::vector<float> state(105, 0.0f);
    
    // 0: Iteraciones sin mejora normalizadas
    state[0] = std::min(1.0f, static_cast<float>(iters_without_improvement) / 2000.0f);
    
    // 1: Progreso total del algoritmo
    state[1] = static_cast<float>(current_iter) / static_cast<float>(max_iters);
    
    // 2: Promedio de utilizacion de capacidad
    double total_load = 0.0;
    int num_routes = 0;
    for(const auto& r : routes) {
        if(r.path.size() > 2) {
            total_load += r.load;
            num_routes++;
        }
    }
    state[2] = (num_routes > 0) ? static_cast<float>(total_load / (num_routes * inst.capacity)) : 0.0f;
    
    // 3: Varianza del tamaño de rutas
    double mean_size = 0.0;
    if(num_routes > 0) {
        for(const auto& r : routes) {
            if(r.path.size() > 2) mean_size += (r.path.size() - 2); // Clientes unicamente
        }
        mean_size /= num_routes;
        
        double var_size = 0.0;
        for(const auto& r : routes) {
            if(r.path.size() > 2) {
                double diff = (r.path.size() - 2) - mean_size;
                var_size += diff * diff;
            }
        }
        var_size /= num_routes;
        state[3] = static_cast<float>(var_size / 100.0); // Normalizado ad hoc
    } else {
        state[3] = 0.0f;
    }
    
    // 4: Fraccion de clientes no asignados
    state[4] = static_cast<float>(unassigned.size()) / 100.0f;
    
    // 5-104: Posicion relativa de cada cliente (1 a 100)
    // Inicializar todo a -1.0 (no asignado)
    for(int i = 5; i < 105; ++i) {
        state[i] = -1.0f;
    }
    
    for(const auto& r : routes) {
        if(r.path.size() > 2) {
            int num_clients = r.path.size() - 2;
            for(size_t i = 1; i < r.path.size() - 1; ++i) {
                int client_idx = r.path[i];
                if(client_idx >= 1 && client_idx <= 100) {
                    // Posicion relativa: de 0.0 (primero) a 1.0 (ultimo)
                    float relative_pos = (num_clients > 1) ? static_cast<float>(i - 1) / (num_clients - 1) : 0.5f;
                    state[4 + client_idx] = relative_pos;
                }
            }
        }
    }
    
    return state;
}