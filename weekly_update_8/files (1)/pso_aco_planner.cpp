/*
 * =============================================================================
 *  pso_aco_planner.cpp
 *
 *  Dynamic Path Planning Based on Improved Ant Colony Algorithm
 *  Implements the PSO-ACO hybrid with Road Condition Factor (R)
 *
 *  Reference: Wu, Zhou & Xiao, IEEE Access 2020, DOI 10.1109/ACCESS.2020.3028467
 *  Flowchart : Uploaded diagram (3.pdf)
 *
 *  Build :  g++ -O2 -std=c++17 -o pso_aco_planner pso_aco_planner.cpp
 *  Run   :  ./pso_aco_planner          (reads config.txt + map_data.csv)
 * =============================================================================
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <limits>
#include <numeric>
#include <iomanip>
#include <map>
#include <cassert>

// =============================================================================
//  SECTION 1 -- Data Structures
// =============================================================================

struct Edge {
    int    from, to;
    double length;       // di  (metres)
    int    lanes;        // li
    double f_current;    // fi(t-1)  vehicles currently on road
    double f_in;         // fin_i(t)
    double f_out;        // fout_i(t)
};

struct Config {
    // Graph
    int    start_node    = 0;
    int    end_node      = 9;
    std::string map_file = "map_data.csv";

    // Vehicle
    double avg_vehicle_len = 5.0;  // L

    // Dynamic trigger
    double R_max       = 250.0;
    double delta_R_max = 30.0;

    // PSO
    int    pso_swarm   = 30;
    int    pso_iter    = 100;
    double pso_w       = 0.7;
    double pso_c1      = 1.5;
    double pso_c2      = 1.5;
    double alpha_min   = 0.5,  alpha_max = 1.5;
    double beta_min    = 1.0,  beta_max  = 5.0;
    double rho_min     = 0.5,  rho_max   = 1.0;

    // ACO
    int    num_ants    = 10;
    int    aco_iter    = 100;
    double Q_const     = 100.0;
    double tau_init    = 1.0;

    // Simulation
    int    sim_steps      = 20;
    double traffic_noise  = 0.2;
    int    random_seed    = 42;
};

// =============================================================================
//  SECTION 2 -- Config Parser
// =============================================================================

static std::string trim(const std::string& s) {
    size_t l = s.find_first_not_of(" \t\r\n");
    size_t r = s.find_last_not_of(" \t\r\n");
    return (l == std::string::npos) ? "" : s.substr(l, r - l + 1);
}

Config load_config(const std::string& path) {
    Config cfg;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[WARN] Cannot open " << path
                  << " -- using built-in defaults.\n";
        return cfg;
    }

    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        // Strip inline comments
        auto hash = val.find('#');
        if (hash != std::string::npos) val = trim(val.substr(0, hash));

        if (key == "START_NODE")      cfg.start_node       = std::stoi(val);
        else if (key == "END_NODE")   cfg.end_node         = std::stoi(val);
        else if (key == "MAP_FILE")   cfg.map_file         = val;
        else if (key == "AVG_VEHICLE_LEN") cfg.avg_vehicle_len = std::stod(val);
        else if (key == "R_MAX")           cfg.R_max        = std::stod(val);
        else if (key == "DELTA_R_MAX")     cfg.delta_R_max  = std::stod(val);
        else if (key == "PSO_SWARM_SIZE")  cfg.pso_swarm    = std::stoi(val);
        else if (key == "PSO_ITERATIONS")  cfg.pso_iter     = std::stoi(val);
        else if (key == "PSO_W")           cfg.pso_w        = std::stod(val);
        else if (key == "PSO_C1")          cfg.pso_c1       = std::stod(val);
        else if (key == "PSO_C2")          cfg.pso_c2       = std::stod(val);
        else if (key == "ALPHA_MIN")       cfg.alpha_min    = std::stod(val);
        else if (key == "ALPHA_MAX")       cfg.alpha_max    = std::stod(val);
        else if (key == "BETA_MIN")        cfg.beta_min     = std::stod(val);
        else if (key == "BETA_MAX")        cfg.beta_max     = std::stod(val);
        else if (key == "RHO_MIN")         cfg.rho_min      = std::stod(val);
        else if (key == "RHO_MAX")         cfg.rho_max      = std::stod(val);
        else if (key == "NUM_ANTS")        cfg.num_ants     = std::stoi(val);
        else if (key == "ACO_ITERATIONS")  cfg.aco_iter     = std::stoi(val);
        else if (key == "Q_CONST")         cfg.Q_const      = std::stod(val);
        else if (key == "TAU_INIT")        cfg.tau_init     = std::stod(val);
        else if (key == "SIM_STEPS")       cfg.sim_steps    = std::stoi(val);
        else if (key == "TRAFFIC_NOISE")   cfg.traffic_noise= std::stod(val);
        else if (key == "RANDOM_SEED")     cfg.random_seed  = std::stoi(val);
    }
    return cfg;
}

// =============================================================================
//  SECTION 3 -- CSV Map Loader
// =============================================================================

std::vector<Edge> load_map(const std::string& path, int& num_nodes) {
    std::vector<Edge> edges;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open map file: " << path << "\n";
        std::exit(1);
    }

    std::string line;
    std::getline(f, line); // skip header
    int max_node = -1;

    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;

        Edge e;
        std::getline(ss, tok, ','); e.from      = std::stoi(tok);
        std::getline(ss, tok, ','); e.to        = std::stoi(tok);
        std::getline(ss, tok, ','); e.length    = std::stod(tok);
        std::getline(ss, tok, ','); e.lanes     = std::stoi(tok);
        std::getline(ss, tok, ','); e.f_current = std::stod(tok);
        std::getline(ss, tok, ','); e.f_in      = std::stod(tok);
        std::getline(ss, tok, ','); e.f_out     = std::stod(tok);

        max_node = std::max(max_node, std::max(e.from, e.to));
        edges.push_back(e);
    }

    num_nodes = max_node + 1;
    return edges;
}

// =============================================================================
//  SECTION 4 -- Road Condition Factor (Paper Section IV-B)
// =============================================================================

/*
 * Congestion coefficient  (formula 8):
 *   Ci(t) = (fi(t-1) + fin_i(t) - fout_i(t)) * L  /  (li * di)
 *           if numerator > 0, else 0
 *
 * Road condition factor   (formula 10):
 *   Ri(t) = di * (1 + Ci(t))
 */
double congestion_coeff(const Edge& e, double L) {
    double net = e.f_current + e.f_in - e.f_out;
    if (net <= 0.0) return 0.0;
    return (net * L) / (static_cast<double>(e.lanes) * e.length);
}

double road_condition_factor(const Edge& e, double L) {
    return e.length * (1.0 + congestion_coeff(e, L));
}

// Build adjacency lookup: adj[from][to] = edge index
std::vector<std::map<int,int>> build_adjacency(
        const std::vector<Edge>& edges, int n)
{
    std::vector<std::map<int,int>> adj(n);
    for (int i = 0; i < (int)edges.size(); ++i)
        adj[edges[i].from][edges[i].to] = i;
    return adj;
}

// =============================================================================
//  SECTION 5 -- Random Utilities
// =============================================================================

static double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

static double rand_range(double lo, double hi) {
    return lo + rand01() * (hi - lo);
}

// =============================================================================
//  SECTION 6 -- ACO (Point-to-Point, Road Condition Factor)
// =============================================================================

struct ACOResult {
    std::vector<int> path;    // sequence of node IDs
    double           cost;    // sum of R values along path
    bool             found;
};

/*
 * Transfer probability (formula 1, adapted for point-to-point):
 *
 *   p_k_ij = [tau_ij]^alpha * [eta_ij]^beta  /  SUM_{s in allowed} [tau_is]^alpha * [eta_is]^beta
 *
 * where eta_ij = 1 / R_ij   (heuristic -- prefer low congestion roads)
 *
 * Ants stop as soon as they reach END_NODE (not when all nodes visited).
 */
ACOResult run_aco(
    const std::vector<Edge>&            edges,
    const std::vector<std::map<int,int>>& adj,
    int    n,
    int    start, int end,
    double alpha, double beta, double rho,
    double Q_const, double tau_init,
    int    num_ants, int max_iter,
    double L)
{
    // Pheromone matrix  tau[i][j] -- using same map structure as adj
    // Stored as n x n dense for simplicity (n <= 100 expected)
    std::vector<std::vector<double>> tau(n, std::vector<double>(n, tau_init));

    ACOResult best;
    best.found = false;
    best.cost  = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < max_iter; ++iter) {

        // Pheromone increment buffer
        std::vector<std::vector<double>> delta_tau(
                n, std::vector<double>(n, 0.0));

        for (int ant = 0; ant < num_ants; ++ant) {
            // -- Ant walk from start -> end --
            std::vector<int> path;
            std::vector<bool> visited(n, false);
            path.push_back(start);
            visited[start] = true;
            int cur = start;
            double cost = 0.0;
            bool reached = false;

            // Safety: at most n*2 steps to avoid infinite loop
            for (int step = 0; step < n * 2 && !reached; ++step) {
                // Collect candidates reachable from cur
                std::vector<int> candidates;
                for (auto it = adj[cur].begin(); it != adj[cur].end(); ++it) {
                    int nb = it->first;
                    if (!visited[nb]) candidates.push_back(nb);
                }
                // If end_node is directly reachable, add even if "visited"
                // (allows direct hop to destination)
                if (!visited[end] && adj[cur].count(end))
                    ; // already in candidates if not visited

                // Allow going to end even if it's in visited
                bool end_direct = adj[cur].count(end) > 0;

                if (candidates.empty() && !end_direct) break; // stuck

                // Build probability distribution
                std::vector<double> probs;
                std::vector<int>    choices;

                auto add_candidate = [&](int nb) {
                    int eidx = adj[cur].at(nb);
                    double R = road_condition_factor(edges[eidx], L);
                    double eta = (R > 1e-9) ? 1.0 / R : 1e9;
                    double val = std::pow(tau[cur][nb], alpha)
                               * std::pow(eta,          beta);
                    probs.push_back(val);
                    choices.push_back(nb);
                };

                // Always include end_node if directly reachable
                bool end_added = false;
                if (end_direct && !visited[end]) {
                    // will be included via candidates
                }
                if (end_direct && adj[cur].count(end)) {
                    // Force-add end as choice even if visited
                    bool already = false;
                    for (int c : candidates)
                        if (c == end) { already = true; break; }
                    if (!already) {
                        candidates.push_back(end);
                        end_added = true;
                    }
                }

                for (int nb : candidates) add_candidate(nb);

                if (choices.empty()) break;

                // Roulette-wheel selection
                double total = 0.0;
                for (double p : probs) total += p;
                double r = rand01() * total;
                double cumul = 0.0;
                int chosen = choices.back();
                for (size_t k = 0; k < choices.size(); ++k) {
                    cumul += probs[k];
                    if (cumul >= r) { chosen = choices[k]; break; }
                }

                int eidx = adj[cur].at(chosen);
                cost += road_condition_factor(edges[eidx], L);
                path.push_back(chosen);
                visited[chosen] = true;
                cur = chosen;

                if (cur == end) { reached = true; }
            }

            if (!reached) continue;

            // -- Pheromone deposit (formula 5) --
            double deposit = Q_const / cost;
            for (size_t k = 0; k + 1 < path.size(); ++k) {
                delta_tau[path[k]][path[k+1]] += deposit;
            }

            if (cost < best.cost) {
                best.cost  = cost;
                best.path  = path;
                best.found = true;
            }
        } // end ants

        // -- Global pheromone evaporation (formula 3) --
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                tau[i][j] = (1.0 - rho) * tau[i][j] + delta_tau[i][j];

    } // end iterations

    return best;
}

// =============================================================================
//  SECTION 7 -- PSO  (tunes alpha, beta, rho for ACO)
// =============================================================================

struct Particle {
    double alpha, beta, rho;
    double v_alpha, v_beta, v_rho;
    double p_alpha, p_beta, p_rho;   // personal best position
    double p_cost;                    // personal best cost
};

struct PSOResult {
    double alpha, beta, rho;
    double best_cost;
};

/*
 * PSO fitness: run ACO with given (alpha, beta, rho) and return best path cost.
 * Lower cost = better.
 * Using formula 6 for velocity update, formula 7 for position update.
 */
PSOResult run_pso(
    const std::vector<Edge>&            edges,
    const std::vector<std::map<int,int>>& adj,
    int n, int start, int end,
    const Config& cfg)
{
    std::vector<Particle> swarm(cfg.pso_swarm);

    // Initialise particles randomly within bounds
    for (auto& p : swarm) {
        p.alpha   = rand_range(cfg.alpha_min, cfg.alpha_max);
        p.beta    = rand_range(cfg.beta_min,  cfg.beta_max);
        p.rho     = rand_range(cfg.rho_min,   cfg.rho_max);
        p.v_alpha = rand_range(-0.1, 0.1);
        p.v_beta  = rand_range(-0.1, 0.1);
        p.v_rho   = rand_range(-0.05, 0.05);
        p.p_alpha = p.alpha;
        p.p_beta  = p.beta;
        p.p_rho   = p.rho;
        p.p_cost  = std::numeric_limits<double>::infinity();
    }

    // Global best
    double g_alpha = swarm[0].alpha;
    double g_beta  = swarm[0].beta;
    double g_rho   = swarm[0].rho;
    double g_cost  = std::numeric_limits<double>::infinity();

    auto clamp = [](double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    };

    std::cout << "\n[PSO] Tuning ACO parameters over "
              << cfg.pso_iter << " iterations ...\n";

    for (int iter = 0; iter < cfg.pso_iter; ++iter) {
        for (auto& p : swarm) {
            // Evaluate fitness
            ACOResult res = run_aco(
                edges, adj, n, start, end,
                p.alpha, p.beta, p.rho,
                cfg.Q_const, cfg.tau_init,
                cfg.num_ants,
                std::max(10, cfg.aco_iter / 5),   // shorter eval run
                cfg.avg_vehicle_len);

            double cost = res.found
                        ? res.cost
                        : std::numeric_limits<double>::infinity();

            // Personal best update
            if (cost < p.p_cost) {
                p.p_cost  = cost;
                p.p_alpha = p.alpha;
                p.p_beta  = p.beta;
                p.p_rho   = p.rho;
            }
            // Global best update
            if (cost < g_cost) {
                g_cost  = cost;
                g_alpha = p.alpha;
                g_beta  = p.beta;
                g_rho   = p.rho;
            }

            // Velocity & position update (formulas 6-7)
            double r1 = rand01(), r2 = rand01();
            p.v_alpha = cfg.pso_w * p.v_alpha
                      + cfg.pso_c1 * r1 * (p.p_alpha - p.alpha)
                      + cfg.pso_c2 * r2 * (g_alpha   - p.alpha);
            p.v_beta  = cfg.pso_w * p.v_beta
                      + cfg.pso_c1 * r1 * (p.p_beta  - p.beta)
                      + cfg.pso_c2 * r2 * (g_beta    - p.beta);
            p.v_rho   = cfg.pso_w * p.v_rho
                      + cfg.pso_c1 * r1 * (p.p_rho   - p.rho)
                      + cfg.pso_c2 * r2 * (g_rho     - p.rho);

            p.alpha = clamp(p.alpha + p.v_alpha, cfg.alpha_min, cfg.alpha_max);
            p.beta  = clamp(p.beta  + p.v_beta,  cfg.beta_min,  cfg.beta_max);
            p.rho   = clamp(p.rho   + p.v_rho,   cfg.rho_min,   cfg.rho_max);
        }

        // Progress every 25 iterations
        if ((iter + 1) % 25 == 0 || iter == 0) {
            std::cout << "  iter " << std::setw(4) << (iter+1)
                      << "  |  best_cost = "
                      << std::fixed << std::setprecision(2) << g_cost
                      << "  alpha=" << std::setprecision(3) << g_alpha
                      << "  beta=" << g_beta
                      << "  rho=" << g_rho << "\n";
        }
    }

    return { g_alpha, g_beta, g_rho, g_cost };
}

// =============================================================================
//  SECTION 8 -- Traffic Simulation (update fi per step)
// =============================================================================

void update_traffic(std::vector<Edge>& edges, double noise) {
    for (auto& e : edges) {
        // Vehicles now on road = previous + in - out
        double net = e.f_current + e.f_in - e.f_out;
        e.f_current = std::max(0.0, net);

        // Randomly vary in/out flows with +/- noise
        double perturb_in  = 1.0 + rand_range(-noise, noise);
        double perturb_out = 1.0 + rand_range(-noise, noise);
        e.f_in  = std::max(0.0, e.f_in  * perturb_in);
        e.f_out = std::max(0.0, e.f_out * perturb_out);
    }
}

// =============================================================================
//  SECTION 9 -- Helpers: printing
// =============================================================================

void print_path(const std::vector<int>& path, double cost,
                const std::string& label)
{
    std::cout << "  " << label << " | cost="
              << std::fixed << std::setprecision(2) << cost
              << "  path: ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i + 1 < path.size()) std::cout << " -> ";
    }
    std::cout << "\n";
}

void print_banner(const std::string& msg) {
    std::string bar(60, '=');
    std::cout << "\n" << bar << "\n  " << msg << "\n" << bar << "\n";
}

double path_cost_from(
    const std::vector<int>& path, int from_node,
    const std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& adj, double L)
{
    double cost = 0.0;
    bool counting = false;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (path[i] == from_node) counting = true;
        if (counting && adj[path[i]].count(path[i+1])) {
            int eidx = adj[path[i]].at(path[i+1]);
            cost += road_condition_factor(edges[eidx], L);
        }
    }
    return cost;
}

// =============================================================================
//  SECTION 10 -- MAIN
// =============================================================================

int main() {
    print_banner("PSO-ACO Dynamic Path Planner  (Wu et al. 2020)");

    // -- Load config --
    Config cfg = load_config("config.txt");

    if (cfg.random_seed != 0) srand(cfg.random_seed);
    else                       srand(static_cast<unsigned>(time(nullptr)));

    std::cout << "\n[CFG] start=" << cfg.start_node
              << "  end=" << cfg.end_node
              << "  map=" << cfg.map_file << "\n";

    // -- Load map --
    int n = 0;
    std::vector<Edge> edges = load_map(cfg.map_file, n);
    auto adj = build_adjacency(edges, n);

    std::cout << "[MAP] " << n << " nodes, "
              << edges.size() << " directed edges loaded.\n";

    // -- Validate start / end --
    if (cfg.start_node >= n || cfg.end_node >= n) {
        std::cerr << "[ERROR] start/end node out of range (max "
                  << n-1 << ").\n";
        return 1;
    }
    if (cfg.start_node == cfg.end_node) {
        std::cerr << "[ERROR] start == end -- nothing to plan.\n";
        return 1;
    }

    // -- Compute initial R values --
    std::cout << "\n[INIT] Initial Road Condition Factors:\n";
    std::cout << "  " << std::setw(12) << "Edge"
              << std::setw(10) << "Length"
              << std::setw(8)  << "Lanes"
              << std::setw(10) << "C(t)"
              << std::setw(10) << "R(t)" << "\n";
    for (const auto& e : edges) {
        double C = congestion_coeff(e, cfg.avg_vehicle_len);
        double R = road_condition_factor(e, cfg.avg_vehicle_len);
        std::cout << "  "
                  << std::setw(4) << e.from << " -> " << std::setw(4) << e.to
                  << std::setw(10) << std::fixed << std::setprecision(1) << e.length
                  << std::setw(8)  << e.lanes
                  << std::setw(10) << std::setprecision(4) << C
                  << std::setw(10) << std::setprecision(2) << R << "\n";
    }

    // =========================================================================
    //  PHASE 1 -- PSO: optimise alpha, beta, rho
    // =========================================================================
    print_banner("PHASE 1 -- PSO Parameter Optimisation");

    PSOResult pso = run_pso(edges, adj, n,
                            cfg.start_node, cfg.end_node, cfg);

    std::cout << "\n[PSO] Optimal ACO parameters found:\n"
              << "      alpha = " << std::fixed << std::setprecision(4) << pso.alpha << "\n"
              << "      beta  = " << pso.beta  << "\n"
              << "      rho   = " << pso.rho   << "\n"
              << "      best_cost = " << pso.best_cost << "\n";

    // =========================================================================
    //  PHASE 2 -- Initial ACO path (besttour)
    // =========================================================================
    print_banner("PHASE 2 -- Initial ACO Path Planning");

    ACOResult initial = run_aco(
        edges, adj, n,
        cfg.start_node, cfg.end_node,
        pso.alpha, pso.beta, pso.rho,
        cfg.Q_const, cfg.tau_init,
        cfg.num_ants, cfg.aco_iter,
        cfg.avg_vehicle_len);

    if (!initial.found) {
        std::cerr << "[ERROR] ACO could not find any path from "
                  << cfg.start_node << " to " << cfg.end_node << ".\n"
                  << "        Check map connectivity.\n";
        return 1;
    }

    std::cout << "\n[ACO] Initial best path found:\n";
    print_path(initial.path, initial.cost, "besttour");

    // =========================================================================
    //  PHASE 3 -- Dynamic Re-planning Loop
    //  (Algorithm steps 5-8 from the paper / flowchart)
    // =========================================================================
    print_banner("PHASE 3 -- Dynamic Simulation & Re-planning");

    std::vector<int> best_path = initial.path;
    double           best_cost = initial.cost;
    double           ar        = 0.0;    // traversed cost so far
    int              cur_node  = cfg.start_node;

    int replan_count = 0;

    // Log file
    std::ofstream log("reroute_log.txt");
    log << "step,cur_node,event,best_cost,ar,path\n";

    auto log_event = [&](int step, const std::string& ev) {
        log << step << "," << cur_node << "," << ev << ","
            << std::fixed << std::setprecision(2)
            << best_cost << "," << ar << ",";
        for (size_t i = 0; i < best_path.size(); ++i) {
            log << best_path[i];
            if (i + 1 < best_path.size()) log << "-";
        }
        log << "\n";
    };

    std::cout << "\nSimulating " << cfg.sim_steps << " time steps ...\n\n";

    for (int step = 0; step < cfg.sim_steps && cur_node != cfg.end_node; ++step) {

        // -- Advance one edge along best_path --
        // Find cur_node's position in path
        int pos = -1;
        for (int i = 0; i < (int)best_path.size(); ++i)
            if (best_path[i] == cur_node) { pos = i; break; }

        if (pos < 0 || pos + 1 >= (int)best_path.size()) {
            std::cout << "[STEP " << step << "] Reached end or lost position.\n";
            break;
        }

        int next_node = best_path[pos + 1];
        int eidx      = -1;
        if (adj[cur_node].count(next_node))
            eidx = adj[cur_node].at(next_node);

        double R_prev = (eidx >= 0)
                       ? road_condition_factor(edges[eidx], cfg.avg_vehicle_len)
                       : 0.0;

        // -- Traffic update (simulate t -> t+1) --
        update_traffic(edges, cfg.traffic_noise);
        std::vector<double> R_new(edges.size());
        for (int i = 0; i < (int)edges.size(); ++i)
            R_new[i] = road_condition_factor(edges[i], cfg.avg_vehicle_len);

        // Rebuild adjacency is not needed (topology unchanged)

        // -- Traverse the segment --
        double R_cur = (eidx >= 0) ? R_new[eidx] : 0.0;
        double delta_R = std::fabs(R_cur - R_prev);

        ar       += R_cur;
        cur_node  = next_node;

        std::cout << "[STEP " << std::setw(2) << step << "] "
                  << "node=" << std::setw(2) << cur_node
                  << "  R=" << std::fixed << std::setprecision(2) << R_cur
                  << "  dR=" << delta_R
                  << "  ar=" << ar;

        // -- Trigger check (formula 11 / flowchart) --
        bool trigger = (delta_R > cfg.delta_R_max) || (R_cur > cfg.R_max);

        if (trigger && cur_node != cfg.end_node) {
            std::cout << "  <- TRIGGER";

            // Re-plan from current node
            ACOResult replan = run_aco(
                edges, adj, n,
                cur_node, cfg.end_node,
                pso.alpha, pso.beta, pso.rho,
                cfg.Q_const, cfg.tau_init,
                cfg.num_ants, cfg.aco_iter,
                cfg.avg_vehicle_len);

            if (replan.found) {
                // Compare: newtour < besttour - ar  (step 7)
               // double remaining_old = best_cost - ar; // old code that was modified
               double remaining_old = path_cost_from(best_path, cur_node, edges, adj, cfg.avg_vehicle_len);
                if (replan.cost < remaining_old) {
                    std::cout << "  -> REROUTE"
                              << " (new=" << replan.cost
                              << " < rem=" << remaining_old << ")";

                    // Accept new plan
                    best_path = replan.path;
                    best_cost = replan.cost + ar;  // besttour = newtour + ar
                    cur_node  = replan.path[0];    // restart from current
                    ++replan_count;

                    log_event(step, "REROUTE");
                } else {
                    std::cout << "  -> keep existing (new="
                              << replan.cost << " >= rem=" << remaining_old << ")";
                    log_event(step, "TRIGGER_KEPT");
                }
            } else {
                std::cout << "  -> no new path found";
                log_event(step, "TRIGGER_NOREPLAN");
            }
        } else {
            log_event(step, "OK");
        }

        std::cout << "\n";

        if (cur_node == cfg.end_node) {
            std::cout << "\n[OK] Destination " << cfg.end_node
                      << " reached at step " << step << "!\n";
            break;
        }
    }

    // =========================================================================
    //  FINAL REPORT
    // =========================================================================
    print_banner("FINAL REPORT");

    std::cout << "  Start Node      : " << cfg.start_node << "\n"
              << "  End Node        : " << cfg.end_node   << "\n"
              << "  Final Node      : " << cur_node       << "\n"
              << "  Reroutes        : " << replan_count   << "\n"
              << "  Total Cost (ar) : "
              << std::fixed << std::setprecision(2) << ar << "\n\n";

    std::cout << "  Final planned path:\n  ";
    print_path(best_path, best_cost, "besttour");

    std::cout << "\n  PSO-tuned params: alpha="
              << std::setprecision(3) << pso.alpha
              << "  beta=" << pso.beta
              << "  rho=" << pso.rho << "\n";

    std::cout << "\n  Log written -> reroute_log.txt\n";
    print_banner("Done.");
    return 0;
}
