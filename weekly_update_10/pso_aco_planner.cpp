/*
 * =============================================================================
 *  pso_aco_planner.cpp
 *
 *  Dynamic Path Planning Based on Improved Ant Colony Algorithm
 *  Implements the PSO-ACO hybrid with Road Condition Factor (R)
 *
 *  Extended for ALL source-destination pairs with network-level analysis.
 *
 *  Reference: Wu, Zhou & Xiao, IEEE Access 2020, DOI 10.1109/ACCESS.2020.3028467
 *  Flowchart : Uploaded diagram (1.png)
 *
 *  Build :  g++ -O2 -std=c++17 -o pso_aco_planner pso_aco_planner.cpp
 *  Run   :  ./pso_aco_planner          (reads config.txt + map_data.csv + sd_pairs.csv)
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
#include <set>
#include <cassert>
#include <utility>

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
    std::string map_file    = "map_data.csv";
    std::string pairs_file  = "sd_pairs.csv";

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

    // Combined Route Optimization
    int    combined_opt       = 1;     // 0=independent only, 1=run combined
    int    combined_iters     = 5;     // max outer-loop iterations
    double combined_weight    = 0.3;   // traffic accumulation factor
    double stability_thresh   = 0.05;  // convergence threshold (5%)
};

// Result from a single source-destination pair run
struct PairResult {
    int source, destination;
    double pso_alpha, pso_beta, pso_rho;
    std::vector<int> initial_path;
    double initial_cost;
    std::vector<int> final_path;
    double final_cost;      // total ar (actual traversed cost)
    int replan_count;
    bool reached;
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
        else if (key == "PAIRS_FILE") cfg.pairs_file       = val;
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
        else if (key == "PAIRS_FILE")      cfg.pairs_file   = val;
        else if (key == "COMBINED_OPT")    cfg.combined_opt      = std::stoi(val);
        else if (key == "COMBINED_ITERS")  cfg.combined_iters    = std::stoi(val);
        else if (key == "COMBINED_WEIGHT") cfg.combined_weight   = std::stod(val);
        else if (key == "STABILITY_THRESH")cfg.stability_thresh  = std::stod(val);
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
//  SECTION 3b -- S-D Pairs Loader
// =============================================================================

std::vector<std::pair<int,int>> load_sd_pairs(const std::string& path) {
    std::vector<std::pair<int,int>> pairs;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open pairs file: " << path << "\n";
        std::exit(1);
    }

    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        int src, dst;
        std::getline(ss, tok, ','); src = std::stoi(trim(tok));
        std::getline(ss, tok, ','); dst = std::stoi(trim(tok));
        pairs.push_back({src, dst});
    }
    return pairs;
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
                std::max(10, cfg.aco_iter / 2),   // evaluation run (half of full)
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

std::string path_to_string(const std::vector<int>& path) {
    std::string s;
    for (size_t i = 0; i < path.size(); ++i) {
        s += std::to_string(path[i]);
        if (i + 1 < path.size()) s += "->";
    }
    return s;
}

std::string path_to_csv_string(const std::vector<int>& path) {
    std::string s;
    for (size_t i = 0; i < path.size(); ++i) {
        s += std::to_string(path[i]);
        if (i + 1 < path.size()) s += "-";
    }
    return s;
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
//  SECTION 10 -- Run Single Source-Destination Pair
//  (Preserves the EXACT flowchart: PSO -> ACO -> Dynamic Re-planning)
// =============================================================================

PairResult run_single_pair(
    std::vector<Edge> edges,                // BY VALUE: fresh traffic per pair
    const std::vector<std::map<int,int>>& adj,
    int n, int src, int dst,
    const Config& cfg,
    std::ofstream& reroute_log,
    int pair_index)
{
    PairResult result;
    result.source      = src;
    result.destination = dst;
    result.replan_count = 0;
    result.reached     = false;

    // =========================================================================
    //  PHASE 1 -- PSO: optimise alpha, beta, rho
    //  (Flowchart: "PHASE 1 -- PSO optimization (one-time)")
    // =========================================================================
    std::cout << "\n  --- PHASE 1: PSO Parameter Optimisation ---\n";

    PSOResult pso = run_pso(edges, adj, n, src, dst, cfg);

    result.pso_alpha = pso.alpha;
    result.pso_beta  = pso.beta;
    result.pso_rho   = pso.rho;

    std::cout << "\n  [PSO] Optimal ACO parameters found:\n"
              << "        alpha = " << std::fixed << std::setprecision(4) << pso.alpha << "\n"
              << "        beta  = " << pso.beta  << "\n"
              << "        rho   = " << pso.rho   << "\n"
              << "        best_cost = " << pso.best_cost << "\n";

    // =========================================================================
    //  PHASE 2 -- Initial ACO path (besttour)
    //  (Flowchart: "PHASE 2 -- Initial pathfinding")
    // =========================================================================
    std::cout << "\n  --- PHASE 2: Initial ACO Path Planning ---\n";

    ACOResult initial = run_aco(
        edges, adj, n,
        src, dst,
        pso.alpha, pso.beta, pso.rho,
        cfg.Q_const, cfg.tau_init,
        cfg.num_ants, cfg.aco_iter,
        cfg.avg_vehicle_len);

    if (!initial.found) {
        std::cerr << "  [WARN] ACO could not find any path from "
                  << src << " to " << dst << ".\n"
                  << "         Check map connectivity.\n";
        result.initial_cost = 0;
        result.final_cost   = 0;
        return result;
    }

    result.initial_path = initial.path;
    result.initial_cost = initial.cost;

    std::cout << "\n  [ACO] Initial best path found:\n";
    print_path(initial.path, initial.cost, "besttour");

    // =========================================================================
    //  PHASE 3 -- Dynamic Re-planning Loop
    //  (Flowchart: "Dynamic monitoring" -> "Trigger condition?" -> etc.)
    // =========================================================================
    std::cout << "\n  --- PHASE 3: Dynamic Simulation & Re-planning ---\n";

    std::vector<int> best_path = initial.path;
    double           best_cost = initial.cost;
    double           ar        = 0.0;    // traversed cost so far
    int              cur_node  = src;

    std::cout << "  Simulating " << cfg.sim_steps << " time steps ...\n\n";

    for (int step = 0; step < cfg.sim_steps && cur_node != dst; ++step) {

        // -- Advance one edge along best_path --
        // Find cur_node's position in path
        int pos = -1;
        for (int i = 0; i < (int)best_path.size(); ++i)
            if (best_path[i] == cur_node) { pos = i; break; }

        if (pos < 0 || pos + 1 >= (int)best_path.size()) {
            std::cout << "  [STEP " << step << "] Reached end or lost position.\n";
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
        // (Flowchart: "Dynamic monitoring (t -> t+1)")
        update_traffic(edges, cfg.traffic_noise);
        std::vector<double> R_new(edges.size());
        for (int i = 0; i < (int)edges.size(); ++i)
            R_new[i] = road_condition_factor(edges[i], cfg.avg_vehicle_len);

        // -- Traverse the segment --
        double R_cur = (eidx >= 0) ? R_new[eidx] : 0.0;
        double delta_R = std::fabs(R_cur - R_prev);

        ar       += R_cur;
        cur_node  = next_node;

        std::cout << "  [STEP " << std::setw(2) << step << "] "
                  << "node=" << std::setw(2) << cur_node
                  << "  R=" << std::fixed << std::setprecision(2) << R_cur
                  << "  dR=" << delta_R
                  << "  ar=" << ar;

        // -- Trigger check (formula 11 / flowchart) --
        // (Flowchart: "Trigger condition? (dR_i > dR_max) OR (R_i(t) > R_max)")
        bool trigger = (delta_R > cfg.delta_R_max) || (R_cur > cfg.R_max);

        if (trigger && cur_node != dst) {
            std::cout << "  <- TRIGGER";

            // Re-plan from current node
            // (Flowchart: "YES: Re-plan from current node using improved ACO")
            ACOResult replan = run_aco(
                edges, adj, n,
                cur_node, dst,
                pso.alpha, pso.beta, pso.rho,
                cfg.Q_const, cfg.tau_init,
                cfg.num_ants, cfg.aco_iter,
                cfg.avg_vehicle_len);

            if (replan.found) {
                // Compare: newtour < besttour - ar  (step 7)
                // (Flowchart: "Is newtour better? newtour < (besttour - a_r)")
                double remaining_old = path_cost_from(best_path, cur_node, edges, adj, cfg.avg_vehicle_len);
                if (replan.cost < remaining_old) {
                    std::cout << "  -> REROUTE"
                              << " (new=" << replan.cost
                              << " < rem=" << remaining_old << ")";

                    // Accept new plan
                    // (Flowchart: "YES update: besttour <- newtour + a_r")
                    best_path = replan.path;
                    best_cost = replan.cost + ar;
                    cur_node  = replan.path[0];
                    ++result.replan_count;

                    // Log
                    reroute_log << pair_index << "," << src << "," << dst << ","
                                << step << "," << cur_node << ",REROUTE,"
                                << std::fixed << std::setprecision(2)
                                << best_cost << "," << ar << ","
                                << path_to_csv_string(best_path) << "\n";
                } else {
                    std::cout << "  -> keep existing (new="
                              << replan.cost << " >= rem=" << remaining_old << ")";
                    reroute_log << pair_index << "," << src << "," << dst << ","
                                << step << "," << cur_node << ",TRIGGER_KEPT,"
                                << std::fixed << std::setprecision(2)
                                << best_cost << "," << ar << ","
                                << path_to_csv_string(best_path) << "\n";
                }
            } else {
                std::cout << "  -> no new path found";
                reroute_log << pair_index << "," << src << "," << dst << ","
                            << step << "," << cur_node << ",TRIGGER_NOREPLAN,"
                            << std::fixed << std::setprecision(2)
                            << best_cost << "," << ar << ","
                            << path_to_csv_string(best_path) << "\n";
            }
        } else {
            reroute_log << pair_index << "," << src << "," << dst << ","
                        << step << "," << cur_node << ",OK,"
                        << std::fixed << std::setprecision(2)
                        << best_cost << "," << ar << ","
                        << path_to_csv_string(best_path) << "\n";
        }

        std::cout << "\n";

        if (cur_node == dst) {
            std::cout << "\n  [OK] Destination " << dst
                      << " reached at step " << step << "!\n";
            break;
        }
    }

    result.final_path = best_path;
    result.final_cost = ar;
    result.reached    = (cur_node == dst);

    return result;
}

// =============================================================================
//  SECTION 10b -- Combined Route Optimization (Iterative)
//  Outer loop: accumulate traffic from shared edges, re-run until stable.
//  Inner loop: run each S-D pair using the EXACT flowchart (PSO->ACO->Replan).
// =============================================================================

struct CombinedIterResult {
    int    iteration;
    double avg_cost;
    double cost_change_pct;   // vs previous iteration
    int    total_reroutes;
    int    route_changes;     // how many pairs changed their final path
};

/// Apply aggregate traffic from route results onto the edge set.
/// For each edge used by N routes, increase f_current by weight * N.
void apply_aggregate_traffic(
    std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& adj,
    const std::vector<PairResult>& results,
    double weight)
{
    // Count edge usage
    std::map<std::pair<int,int>, int> edge_freq;
    for (const auto& r : results) {
        if (!r.reached) continue;
        const auto& path = r.final_path;
        for (size_t k = 0; k + 1 < path.size(); ++k)
            edge_freq[{path[k], path[k+1]}]++;
    }

    // Apply: for each used edge, add weight * freq to f_current and f_in
    for (auto& e : edges) {
        auto key = std::make_pair(e.from, e.to);
        auto it = edge_freq.find(key);
        if (it != edge_freq.end() && it->second >= 2) {
            // Only penalise edges shared by 2+ routes
            double penalty = weight * (it->second - 1);
            e.f_current += penalty * (e.lanes * e.length / 5.0) * 0.1;
            e.f_in      += penalty * 2.0;
        }
    }
}

std::pair<std::vector<PairResult>, std::vector<CombinedIterResult>>
run_combined_optimization(
    const std::vector<Edge>& original_edges,
    const std::vector<std::map<int,int>>& adj,
    int n,
    const std::vector<std::pair<int,int>>& sd_pairs,
    const Config& cfg)
{
    std::vector<CombinedIterResult> history;
    std::vector<PairResult> best_results;
    std::vector<PairResult> prev_results;
    double prev_avg_cost = 0.0;

    // Open a combined reroute log
    std::ofstream combined_rlog("reroute_log_combined.txt");
    combined_rlog << "combined_iter,pair_id,source,destination,step,cur_node,"
                  << "event,best_cost,ar,path\n";

    for (int citer = 0; citer < cfg.combined_iters; ++citer) {
        print_banner("COMBINED ITERATION " + std::to_string(citer + 1)
                     + " / " + std::to_string(cfg.combined_iters));

        // Build traffic-adjusted edges for this iteration
        std::vector<Edge> iter_edges = original_edges;
        if (citer > 0 && !prev_results.empty()) {
            // Accumulate traffic from previous iteration's routes
            apply_aggregate_traffic(iter_edges, adj, prev_results, cfg.combined_weight);
            std::cout << "  [COMBINED] Applied aggregate traffic from iteration "
                      << citer << " (weight=" << cfg.combined_weight << ")\n";
        }

        // Run all S-D pairs with the adjusted traffic
        std::vector<PairResult> iter_results;
        for (int i = 0; i < (int)sd_pairs.size(); ++i) {
            int src = sd_pairs[i].first;
            int dst = sd_pairs[i].second;

            std::string pair_label = "COMB-ITER " + std::to_string(citer+1)
                                   + "  PAIR " + std::to_string(i)
                                   + " (" + std::to_string(src)
                                   + " -> " + std::to_string(dst) + ")";
            print_banner(pair_label);

            // Wrap reroute log to prefix combined iteration
            std::ofstream pair_rlog("reroute_log.txt", std::ios::app);

            PairResult result = run_single_pair(
                iter_edges, adj, n, src, dst, cfg, pair_rlog, i);

            pair_rlog.close();

            iter_results.push_back(result);

            std::cout << "\n  Pair " << i << " " << src << "->" << dst
                      << " cost=" << std::fixed << std::setprecision(2)
                      << result.final_cost
                      << " reached=" << (result.reached ? "YES" : "NO") << "\n";
        }

        // Compute iteration statistics
        double total_cost = 0.0;
        int    successful = 0;
        int    total_reroutes = 0;
        int    route_changes = 0;

        for (int i = 0; i < (int)iter_results.size(); ++i) {
            const auto& r = iter_results[i];
            if (r.reached) {
                total_cost += r.final_cost;
                successful++;
            }
            total_reroutes += r.replan_count;

            // Check if route changed from previous iteration
            if (citer > 0 && i < (int)prev_results.size()) {
                if (iter_results[i].final_path != prev_results[i].final_path)
                    route_changes++;
            }
        }

        double avg_cost = (successful > 0) ? total_cost / successful : 0.0;
        double change_pct = (citer > 0 && prev_avg_cost > 0)
                          ? std::fabs(avg_cost - prev_avg_cost) / prev_avg_cost
                          : 1.0;

        CombinedIterResult cir;
        cir.iteration      = citer + 1;
        cir.avg_cost        = avg_cost;
        cir.cost_change_pct = change_pct;
        cir.total_reroutes  = total_reroutes;
        cir.route_changes   = (citer > 0) ? route_changes : (int)sd_pairs.size();
        history.push_back(cir);

        std::cout << "\n  [COMBINED ITER " << citer+1 << "] avg_cost="
                  << std::fixed << std::setprecision(2) << avg_cost
                  << "  change=" << std::setprecision(2) << (change_pct*100) << "%"
                  << "  route_changes=" << route_changes
                  << "  reroutes=" << total_reroutes << "\n";

        prev_results  = iter_results;
        best_results  = iter_results;
        prev_avg_cost = avg_cost;

        // Stabilization check
        if (citer > 0 && change_pct < cfg.stability_thresh) {
            std::cout << "\n  ** STABILIZED at iteration " << citer+1
                      << " (change " << std::setprecision(2) << (change_pct*100)
                      << "% < threshold " << (cfg.stability_thresh*100) << "%) **\n";
            break;
        }
    }

    combined_rlog.close();
    return {best_results, history};
}

// =============================================================================
//  SECTION 11 -- Save All-Pairs Results to CSV
// =============================================================================

void save_results(const std::vector<PairResult>& results,
                  const std::string& filename)
{
    std::ofstream f(filename);
    f << "pair_id,source,destination,initial_path,initial_cost,"
      << "final_path,final_cost,reroutes,reached,"
      << "pso_alpha,pso_beta,pso_rho\n";

    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        f << i << ","
          << r.source << "," << r.destination << ","
          << path_to_csv_string(r.initial_path) << ","
          << std::fixed << std::setprecision(2) << r.initial_cost << ","
          << path_to_csv_string(r.final_path) << ","
          << r.final_cost << ","
          << r.replan_count << ","
          << (r.reached ? "YES" : "NO") << ","
          << std::setprecision(4) << r.pso_alpha << ","
          << r.pso_beta << "," << r.pso_rho << "\n";
    }
    f.close();
    std::cout << "\n  Results saved -> " << filename << "\n";
}

// =============================================================================
//  SECTION 12 -- Network-Level Analysis (Combined Route Optimization)
// =============================================================================

void analyze_network(const std::vector<PairResult>& results,
                     const std::vector<Edge>& edges,
                     int n,
                     const std::string& filename)
{
    std::ofstream f(filename);

    f << "================================================================\n"
      << "  NETWORK-LEVEL ANALYSIS (Combined Route Optimization)\n"
      << "================================================================\n\n";

    // --- Edge usage frequency ---
    std::map<std::pair<int,int>, int> edge_freq;
    std::map<std::pair<int,int>, std::vector<int>> edge_users; // which pairs use each edge
    std::map<int, int> node_freq;
    std::map<int, std::vector<int>> node_users;

    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        if (!r.reached) continue;
        const auto& path = r.final_path;

        // Count nodes
        for (int nd : path) {
            node_freq[nd]++;
            node_users[nd].push_back(i);
        }

        // Count edges
        for (size_t k = 0; k + 1 < path.size(); ++k) {
            auto key = std::make_pair(path[k], path[k+1]);
            edge_freq[key]++;
            edge_users[key].push_back(i);
        }
    }

    // --- Print edge frequency table ---
    f << "1. EDGE USAGE FREQUENCY (across all successful routes)\n"
      << "   (Higher frequency = potential bottleneck in combined optimization)\n\n";
    f << std::setw(12) << "Edge" << std::setw(10) << "Freq"
      << "    Used by pairs\n";
    f << std::string(60, '-') << "\n";

    // Sort by frequency descending
    std::vector<std::pair<std::pair<int,int>, int>> sorted_edges(
        edge_freq.begin(), edge_freq.end());
    std::sort(sorted_edges.begin(), sorted_edges.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    for (size_t ei = 0; ei < sorted_edges.size(); ++ei) {
        const auto& edge = sorted_edges[ei].first;
        int freq = sorted_edges[ei].second;
        f << "   " << std::setw(2) << edge.first << " -> "
          << std::setw(2) << edge.second
          << std::setw(10) << freq << "    [";
        const auto& users = edge_users[edge];
        for (size_t i = 0; i < users.size(); ++i) {
            f << "P" << users[i];
            if (i + 1 < users.size()) f << ",";
        }
        f << "]\n";
    }

    // --- Print node frequency table ---
    f << "\n2. NODE USAGE FREQUENCY (across all successful routes)\n"
      << "   (Higher frequency = critical intersection in the network)\n\n";
    f << std::setw(8) << "Node" << std::setw(10) << "Freq"
      << "    Used by pairs\n";
    f << std::string(60, '-') << "\n";

    std::vector<std::pair<int, int>> sorted_nodes(
        node_freq.begin(), node_freq.end());
    std::sort(sorted_nodes.begin(), sorted_nodes.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    for (size_t ni = 0; ni < sorted_nodes.size(); ++ni) {
        int node = sorted_nodes[ni].first;
        int freq = sorted_nodes[ni].second;
        f << "   " << std::setw(4) << node
          << std::setw(10) << freq << "    [";
        const auto& users = node_users[node];
        for (size_t i = 0; i < users.size(); ++i) {
            f << "P" << users[i];
            if (i + 1 < users.size()) f << ",";
        }
        f << "]\n";
    }

    // --- Shared edges (used by 2+ routes) ---
    f << "\n3. SHARED EDGES (used by 2+ routes -- combined route overlap)\n"
      << "   These edges represent the network segments where multiple S-D\n"
      << "   routes overlap. In combined route optimization, these edges\n"
      << "   would experience higher aggregate traffic load.\n\n";

    int shared_count = 0;
    for (size_t ei = 0; ei < sorted_edges.size(); ++ei) {
        const auto& edge = sorted_edges[ei].first;
        int freq = sorted_edges[ei].second;
        if (freq >= 2) {
            f << "   Edge " << edge.first << " -> " << edge.second
              << "  shared by " << freq << " routes: ";
            const auto& users = edge_users[edge];
            for (size_t i = 0; i < users.size(); ++i) {
                const auto& r = results[users[i]];
                f << "(" << r.source << "->" << r.destination << ")";
                if (i + 1 < users.size()) f << ", ";
            }
            f << "\n";
            ++shared_count;
        }
    }
    if (shared_count == 0) {
        f << "   No shared edges detected.\n";
    }

    // --- Shared nodes (used by 3+ routes) ---
    f << "\n4. CRITICAL NODES (used by 3+ routes -- key intersections)\n\n";
    int critical_count = 0;
    for (size_t ni = 0; ni < sorted_nodes.size(); ++ni) {
        int node = sorted_nodes[ni].first;
        int freq = sorted_nodes[ni].second;
        if (freq >= 3) {
            f << "   Node " << node << "  used by " << freq << " routes: ";
            const auto& users = node_users[node];
            for (size_t i = 0; i < users.size(); ++i) {
                const auto& r = results[users[i]];
                f << "(" << r.source << "->" << r.destination << ")";
                if (i + 1 < users.size()) f << ", ";
            }
            f << "\n";
            ++critical_count;
        }
    }
    if (critical_count == 0) {
        f << "   No critical nodes detected (threshold: 3+ routes).\n";
    }

    // --- Stabilized route summary ---
    f << "\n5. STABILIZED ROUTE SUMMARY\n"
      << "   Final routes after dynamic re-planning for all S-D pairs:\n\n";

    int total_reroutes = 0;
    int total_reached  = 0;
    double total_cost  = 0;
    int successful     = 0;

    f << std::setw(5)  << "Pair"
      << std::setw(8)  << "Src"
      << std::setw(8)  << "Dst"
      << std::setw(12) << "Init Cost"
      << std::setw(12) << "Final Cost"
      << std::setw(10) << "Reroutes"
      << std::setw(10) << "Reached"
      << "  Final Path\n";
    f << std::string(100, '-') << "\n";

    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        f << std::setw(5) << i
          << std::setw(8) << r.source
          << std::setw(8) << r.destination
          << std::setw(12) << std::fixed << std::setprecision(2) << r.initial_cost
          << std::setw(12) << r.final_cost
          << std::setw(10) << r.replan_count
          << std::setw(10) << (r.reached ? "YES" : "NO")
          << "  " << path_to_string(r.final_path) << "\n";

        total_reroutes += r.replan_count;
        if (r.reached) {
            total_reached++;
            total_cost += r.final_cost;
            successful++;
        }
    }

    f << std::string(100, '-') << "\n";
    f << "\n  SUMMARY STATISTICS:\n"
      << "    Total S-D pairs processed : " << results.size() << "\n"
      << "    Successfully reached      : " << total_reached << " / " << results.size() << "\n"
      << "    Total reroutes across all  : " << total_reroutes << "\n"
      << "    Average final cost (succ.) : "
      << (successful > 0 ? total_cost / successful : 0.0) << "\n"
      << "    Total unique edges used    : " << edge_freq.size() << "\n"
      << "    Shared edges (2+ routes)   : " << shared_count << "\n"
      << "    Total unique nodes used    : " << node_freq.size() << "\n"
      << "    Critical nodes (3+ routes) : " << critical_count << "\n";

    // --- Combined route optimization insight ---
    f << "\n6. COMBINED ROUTE OPTIMIZATION INSIGHT\n\n"
      << "   In a pure independent optimization, each S-D pair is solved\n"
      << "   without considering traffic from other pairs. The analysis above\n"
      << "   shows which edges/nodes are over-utilized when routes are computed\n"
      << "   independently.\n\n"
      << "   Key observation for combined optimization:\n"
      << "   - Edges with frequency >= 2 would benefit from load balancing\n"
      << "   - Critical nodes (freq >= 3) are bottleneck intersections\n"
      << "   - If routes were computed jointly (considering aggregate traffic),\n"
      << "     some routes would shift to alternative paths to reduce\n"
      << "     congestion on shared segments.\n\n"
      << "   This forms the basis for iterative combined optimization:\n"
      << "   1. Compute all routes independently (current approach)\n"
      << "   2. Measure edge/node frequencies\n"
      << "   3. Increase traffic weights on high-frequency edges\n"
      << "   4. Re-optimize routes with updated weights\n"
      << "   5. Repeat until route assignment stabilizes\n";

    f << "\n================================================================\n"
      << "  End of Network Analysis\n"
      << "================================================================\n";

    f.close();
    std::cout << "  Network analysis saved -> " << filename << "\n";
}

// =============================================================================
//  SECTION 12b -- Stabilization Analysis & Comparison
// =============================================================================

void analyze_stabilization(
    const std::vector<PairResult>& independent_results,
    const std::vector<PairResult>& combined_results,
    const std::vector<CombinedIterResult>& history,
    const std::string& filename)
{
    std::ofstream f(filename);

    f << "================================================================\n"
      << "  COMBINED ROUTE OPTIMIZATION: COMPARISON REPORT\n"
      << "================================================================\n\n";

    // --- Convergence History ---
    f << "1. CONVERGENCE HISTORY\n"
      << "   Shows how average route cost changes across combined iterations.\n\n";
    f << std::setw(10) << "Iter"
      << std::setw(14) << "Avg Cost"
      << std::setw(14) << "Change %"
      << std::setw(14) << "Reroutes"
      << std::setw(16) << "Route Changes" << "\n";
    f << std::string(68, '-') << "\n";

    for (const auto& h : history) {
        f << std::setw(10) << h.iteration
          << std::setw(14) << std::fixed << std::setprecision(2) << h.avg_cost
          << std::setw(13) << std::setprecision(2) << (h.cost_change_pct * 100) << "%"
          << std::setw(14) << h.total_reroutes
          << std::setw(16) << h.route_changes << "\n";
    }

    // --- Per-Pair Comparison ---
    f << "\n2. PER-PAIR COMPARISON: INDEPENDENT vs COMBINED\n"
      << "   Shows cost difference and route changes per S-D pair.\n\n";
    f << std::setw(5)  << "Pair"
      << std::setw(6)  << "Src"
      << std::setw(6)  << "Dst"
      << std::setw(14) << "Indep Cost"
      << std::setw(14) << "Comb Cost"
      << std::setw(12) << "Diff %"
      << std::setw(14) << "Route Same?"
      << "  Independent Path -> Combined Path\n";
    f << std::string(110, '-') << "\n";

    double total_indep = 0, total_comb = 0;
    int improved = 0, worsened = 0, same_route = 0;
    int indep_succ = 0, comb_succ = 0;

    int count = std::min(independent_results.size(), combined_results.size());
    for (int i = 0; i < count; ++i) {
        const auto& ir = independent_results[i];
        const auto& cr = combined_results[i];

        double diff_pct = 0;
        if (ir.reached && ir.final_cost > 0)
            diff_pct = (cr.final_cost - ir.final_cost) / ir.final_cost * 100;

        bool routes_same = (ir.final_path == cr.final_path);
        if (routes_same) same_route++;

        if (ir.reached) { total_indep += ir.final_cost; indep_succ++; }
        if (cr.reached) { total_comb  += cr.final_cost; comb_succ++;  }
        if (cr.reached && ir.reached) {
            if (cr.final_cost < ir.final_cost) improved++;
            else if (cr.final_cost > ir.final_cost) worsened++;
        }

        f << std::setw(5)  << i
          << std::setw(6)  << ir.source
          << std::setw(6)  << ir.destination
          << std::setw(14) << std::fixed << std::setprecision(2)
          << (ir.reached ? ir.final_cost : 0.0)
          << std::setw(14) << (cr.reached ? cr.final_cost : 0.0)
          << std::setw(11) << std::setprecision(1) << diff_pct << "%"
          << std::setw(14) << (routes_same ? "YES" : "NO")
          << "  " << path_to_string(ir.final_path)
          << " -> " << path_to_string(cr.final_path) << "\n";
    }

    f << std::string(110, '-') << "\n";

    // --- Summary ---
    f << "\n3. OVERALL COMPARISON SUMMARY\n\n";
    f << "   Independent optimization:\n"
      << "     Successful routes : " << indep_succ << " / " << count << "\n"
      << "     Average cost      : " << std::fixed << std::setprecision(2)
      << (indep_succ > 0 ? total_indep / indep_succ : 0.0) << "\n\n";
    f << "   Combined optimization (final iteration):\n"
      << "     Successful routes : " << comb_succ << " / " << count << "\n"
      << "     Average cost      : "
      << (comb_succ > 0 ? total_comb / comb_succ : 0.0) << "\n\n";
    f << "   Pairs improved (lower cost with combined) : " << improved << "\n"
      << "   Pairs worsened (higher cost)              : " << worsened << "\n"
      << "   Pairs with same final route               : " << same_route
      << " / " << count << "\n";

    double overall_change = 0;
    if (indep_succ > 0) {
        double avg_i = total_indep / indep_succ;
        double avg_c = (comb_succ > 0) ? total_comb / comb_succ : avg_i;
        overall_change = (avg_c - avg_i) / avg_i * 100;
    }
    f << "   Overall avg cost change                   : "
      << std::setprecision(2) << overall_change << "%\n";

    f << "\n   Interpretation:\n";
    if (overall_change < -1.0) {
        f << "   -> Combined optimization REDUCED average cost.\n"
          << "      Shared-edge traffic consideration led to better path diversity.\n";
    } else if (overall_change > 1.0) {
        f << "   -> Combined optimization INCREASED average cost.\n"
          << "      This is expected: routes now account for aggregate traffic,\n"
          << "      yielding more realistic (but costlier) individual paths.\n"
          << "      The NETWORK as a whole is more balanced.\n";
    } else {
        f << "   -> Combined and independent results are nearly identical.\n"
          << "      The network has sufficient capacity for independent routing.\n";
    }

    f << "\n   Combined iterations until stabilization : " << history.size() << "\n";
    if (!history.empty()) {
        f << "   Final convergence change                : "
          << std::setprecision(2) << (history.back().cost_change_pct * 100) << "%\n";
    }

    f << "\n================================================================\n"
      << "  End of Comparison Report\n"
      << "================================================================\n";

    f.close();
    std::cout << "  Comparison report saved -> " << filename << "\n";
}

// =============================================================================
//  SECTION 13 -- MAIN
// =============================================================================

static void print_summary_table(
    const std::vector<PairResult>& results, const std::string& label)
{
    std::cout << "\n  " << std::setw(5) << "Pair"
              << std::setw(6)  << "Src"
              << std::setw(6)  << "Dst"
              << std::setw(11) << "InitCost"
              << std::setw(11) << "FinalCost"
              << std::setw(9)  << "Reroutes"
              << std::setw(9)  << "Reached"
              << "  Final Path\n";
    std::cout << "  " << std::string(80, '-') << "\n";

    int total_reroutes = 0, total_reached = 0;
    double total_cost = 0;

    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        std::cout << "  " << std::setw(5) << i
                  << std::setw(6) << r.source
                  << std::setw(6) << r.destination
                  << std::setw(11) << std::fixed << std::setprecision(2) << r.initial_cost
                  << std::setw(11) << r.final_cost
                  << std::setw(9)  << r.replan_count
                  << std::setw(9)  << (r.reached ? "YES" : "NO")
                  << "  " << path_to_string(r.final_path) << "\n";

        total_reroutes += r.replan_count;
        if (r.reached) { total_reached++; total_cost += r.final_cost; }
    }

    std::cout << "\n  Total pairs    : " << results.size()
              << "\n  Reached dest.  : " << total_reached << " / " << results.size()
              << "\n  Total reroutes : " << total_reroutes
              << "\n  Avg final cost : "
              << (total_reached > 0 ? total_cost / total_reached : 0.0) << "\n";
}

int main() {
    print_banner("PSO-ACO Dynamic Path Planner  (Wu et al. 2020)");
    std::cout << "  Extended: ALL S-D Pairs + Combined Route Optimization\n";

    // -- Load config --
    Config cfg = load_config("config.txt");

    if (cfg.random_seed != 0) srand(cfg.random_seed);
    else                       srand(static_cast<unsigned>(time(nullptr)));

    std::cout << "\n[CFG] map=" << cfg.map_file
              << "  pairs_file=" << cfg.pairs_file
              << "  combined_opt=" << cfg.combined_opt << "\n";

    // -- Load map --
    int n = 0;
    std::vector<Edge> original_edges = load_map(cfg.map_file, n);
    auto adj = build_adjacency(original_edges, n);

    std::cout << "[MAP] " << n << " nodes, "
              << original_edges.size() << " directed edges loaded.\n";

    // -- Compute initial R values --
    std::cout << "\n[INIT] Initial Road Condition Factors:\n";
    std::cout << "  " << std::setw(12) << "Edge"
              << std::setw(10) << "Length"
              << std::setw(8)  << "Lanes"
              << std::setw(10) << "C(t)"
              << std::setw(10) << "R(t)" << "\n";
    for (const auto& e : original_edges) {
        double C = congestion_coeff(e, cfg.avg_vehicle_len);
        double R = road_condition_factor(e, cfg.avg_vehicle_len);
        std::cout << "  "
                  << std::setw(4) << e.from << " -> " << std::setw(4) << e.to
                  << std::setw(10) << std::fixed << std::setprecision(1) << e.length
                  << std::setw(8)  << e.lanes
                  << std::setw(10) << std::setprecision(4) << C
                  << std::setw(10) << std::setprecision(2) << R << "\n";
    }

    // -- Load S-D pairs --
    std::vector<std::pair<int,int>> sd_pairs = load_sd_pairs(cfg.pairs_file);
    std::cout << "\n[PAIRS] " << sd_pairs.size()
              << " source-destination pairs loaded from " << cfg.pairs_file << ":\n";
    for (int i = 0; i < (int)sd_pairs.size(); ++i) {
        std::cout << "  P" << i << ": " << sd_pairs[i].first
                  << " -> " << sd_pairs[i].second << "\n";
    }

    // -- Validate pairs --
    for (int i = 0; i < (int)sd_pairs.size(); ++i) {
        int src = sd_pairs[i].first, dst = sd_pairs[i].second;
        if (src >= n || dst >= n || src < 0 || dst < 0) {
            std::cerr << "[ERROR] Pair P" << i << " (" << src << "->" << dst
                      << ") has node out of range (max " << n-1 << ").\n";
            return 1;
        }
        if (src == dst) {
            std::cerr << "[ERROR] Pair P" << i << " has src == dst ("
                      << src << "). Skipping is not supported.\n";
            return 1;
        }
    }

    // =========================================================================
    //  STEP A: Independent optimization (original behavior)
    //  Each pair runs the FULL flowchart: PSO -> ACO -> Dynamic Re-planning
    // =========================================================================
    print_banner("STEP A: INDEPENDENT OPTIMIZATION");

    std::ofstream reroute_log("reroute_log.txt");
    reroute_log << "pair_id,source,destination,step,cur_node,event,"
                << "best_cost,ar,path\n";

    std::vector<PairResult> independent_results;

    for (int i = 0; i < (int)sd_pairs.size(); ++i) {
        int src = sd_pairs[i].first;
        int dst = sd_pairs[i].second;

        std::string pair_label = "PAIR " + std::to_string(i)
                               + " (" + std::to_string(src)
                               + " -> " + std::to_string(dst) + ")";
        print_banner(pair_label);

        PairResult result = run_single_pair(
            original_edges, adj, n, src, dst, cfg, reroute_log, i);

        independent_results.push_back(result);

        std::cout << "\n  --- PAIR " << i << " FINAL REPORT ---\n"
                  << "  Source        : " << src << "\n"
                  << "  Destination   : " << dst << "\n"
                  << "  Reached       : " << (result.reached ? "YES" : "NO") << "\n"
                  << "  Reroutes      : " << result.replan_count << "\n"
                  << "  Initial cost  : " << std::fixed << std::setprecision(2)
                  << result.initial_cost << "\n"
                  << "  Final cost(ar): " << result.final_cost << "\n"
                  << "  Initial path  : " << path_to_string(result.initial_path) << "\n"
                  << "  Final path    : " << path_to_string(result.final_path) << "\n"
                  << "  PSO params    : alpha=" << std::setprecision(3) << result.pso_alpha
                  << "  beta=" << result.pso_beta
                  << "  rho=" << result.pso_rho << "\n";
    }

    reroute_log.close();

    // -- Independent summary --
    print_banner("INDEPENDENT OPTIMIZATION SUMMARY");
    print_summary_table(independent_results, "Independent");

    save_results(independent_results, "all_pairs_results.csv");
    analyze_network(independent_results, original_edges, n, "network_analysis.txt");

    // =========================================================================
    //  STEP B: Combined Route Optimization (if enabled)
    //  Outer loop accumulates shared-edge traffic, re-runs until stable.
    // =========================================================================
    std::vector<PairResult> combined_results;
    std::vector<CombinedIterResult> combined_history;

    if (cfg.combined_opt) {
        print_banner("STEP B: COMBINED ROUTE OPTIMIZATION");
        std::cout << "  Max iterations    : " << cfg.combined_iters << "\n"
                  << "  Traffic weight    : " << cfg.combined_weight << "\n"
                  << "  Stability thresh  : " << (cfg.stability_thresh * 100) << "%\n";

        auto combined_pair = run_combined_optimization(
            original_edges, adj, n, sd_pairs, cfg);

        combined_results = combined_pair.first;
        combined_history = combined_pair.second;

        // -- Combined summary --
        print_banner("COMBINED OPTIMIZATION SUMMARY (Final Iteration)");
        print_summary_table(combined_results, "Combined");

        save_results(combined_results, "combined_pairs_results.csv");
        analyze_network(combined_results, original_edges, n, "network_analysis_combined.txt");

        // -- Comparison report --
        analyze_stabilization(
            independent_results, combined_results,
            combined_history, "combined_comparison.txt");
    }

    // -- Final output listing --
    std::cout << "\n  Reroute log (independent) -> reroute_log.txt\n";
    std::cout << "  Results CSV (independent) -> all_pairs_results.csv\n";
    std::cout << "  Network analysis          -> network_analysis.txt\n";
    if (cfg.combined_opt) {
        std::cout << "  Results CSV (combined)    -> combined_pairs_results.csv\n";
        std::cout << "  Network analysis (comb.)  -> network_analysis_combined.txt\n";
        std::cout << "  Comparison report         -> combined_comparison.txt\n";
    }

    print_banner("Done. All source-destination pairs processed.");
    return 0;
}
