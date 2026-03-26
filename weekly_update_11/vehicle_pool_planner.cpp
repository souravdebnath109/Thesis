/*
 * =============================================================================
 *  vehicle_pool_planner.cpp
 *
 *  Vehicle-Aware Pool-Based Alternative Path-Planning Framework for Dhaka
 *  Extends: Improved ACO with Road Condition Factor (Wu et al. 2020)
 *  Adds: Vehicle categories, road hierarchy, pool-based rerouting,
 *        online disruption response (Case 1/2/3 comparison)
 *
 *  Follows: Detailed Combined Supervisor Flowchart (Step A, B, C)
 * =============================================================================
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <numeric>
#include <limits>
#include <iomanip>
#include <cassert>
#include <chrono>
#include <utility>

// =============================================================================
//  SECTION 1 -- Data Structures
// =============================================================================

struct Edge {
    int    from, to;
    double length;       // di (metres)
    int    lanes;        // li
    double f_current;    // fi(t-1)
    double f_in;         // fin_i(t)
    double f_out;        // fout_i(t)
    int    road_class;   // 1=main,2=link,3=sub,4=mini
    double width;        // metres
    double road_condition; // 0.0=perfect..1.0=worst
    int    cat_allowed[4]; // cat1..cat4
    int    bus_corridor;   // 0 or 1
    double disruption_prior; // 0..1
};

struct Config {
    std::string map_file      = "map_data.csv";
    std::string pairs_file    = "sd_pairs.csv";
    std::string disruption_file = "disruption_events.csv";
    double avg_vehicle_len    = 5.0;
    int    vehicle_categories = 4;
    double cat_min_width[4]   = {6.0, 5.0, 3.5, 2.0};
    int    pool_size          = 5;
    double R_max              = 150.0;
    double delta_R_max        = 5.0;
    double fixed_alpha        = 1.0;
    double fixed_beta         = 2.0;
    double fixed_rho          = 0.5;
    int    num_ants           = 10;
    int    aco_iter           = 100;
    double Q_const            = 100.0;
    double tau_init           = 1.0;
    int    sim_steps          = 20;
    double traffic_noise      = 0.5;
    int    random_seed        = 42;
    int    combined_opt       = 1;
    int    combined_iters     = 5;
    double combined_weight    = 0.3;
    double stability_thresh   = 0.05;
    double disruption_block_weight = 1000.0;
    int    disruption_congest_radius = 2;
    double congestion_increase = 3.0;
};

struct PoolEntry {
    std::vector<int> path;
    double cost;
    bool   feasible;
};

struct PairResult {
    int source, destination;
    int vehicle_cat;         // 0-based index (0=truck,1=bus,2=car,3=rickshaw)
    std::vector<int> initial_path;
    double initial_cost;
    std::vector<int> final_path;
    double final_cost;
    int    replan_count;
    bool   reached;
    std::vector<PoolEntry> pool; // pool of alternative paths
};

struct DisruptionEvent {
    int    event_id;
    int    blocked_from, blocked_to;
    std::string event_type;
    std::string description;
};

struct CaseResult {
    int    pair_idx;
    int    source, destination;
    int    vehicle_cat;
    int    case_num;
    int    reroute_iterations;
    double reroute_time_ms;
    double energy_proxy;
    bool   success;
    double final_cost;
    std::vector<int> reroute_path;
};

struct ACOResult {
    std::vector<int> path;
    double cost;
    bool   found;
};

struct CombinedIterResult {
    int    iteration;
    double avg_cost;
    double cost_change_pct;
    int    total_reroutes;
    int    route_changes;
    int    pool_changes;
};

// =============================================================================
//  SECTION 2 -- Config Parser
// =============================================================================

static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
}

Config load_config(const std::string& path) {
    Config cfg;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[WARN] Cannot open " << path << ", using defaults.\n";
        return cfg;
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        // strip inline comment
        auto cpos = val.find('#');
        if (cpos != std::string::npos) val = trim(val.substr(0, cpos));

        if      (key == "MAP_FILE")          cfg.map_file = val;
        else if (key == "PAIRS_FILE")        cfg.pairs_file = val;
        else if (key == "DISRUPTION_FILE")   cfg.disruption_file = val;
        else if (key == "AVG_VEHICLE_LEN")   cfg.avg_vehicle_len = std::stod(val);
        else if (key == "VEHICLE_CATEGORIES")cfg.vehicle_categories = std::stoi(val);
        else if (key == "CAT1_MIN_WIDTH")    cfg.cat_min_width[0] = std::stod(val);
        else if (key == "CAT2_MIN_WIDTH")    cfg.cat_min_width[1] = std::stod(val);
        else if (key == "CAT3_MIN_WIDTH")    cfg.cat_min_width[2] = std::stod(val);
        else if (key == "CAT4_MIN_WIDTH")    cfg.cat_min_width[3] = std::stod(val);
        else if (key == "POOL_SIZE")         cfg.pool_size = std::stoi(val);
        else if (key == "R_MAX")             cfg.R_max = std::stod(val);
        else if (key == "DELTA_R_MAX")       cfg.delta_R_max = std::stod(val);
        else if (key == "FIXED_ALPHA")       cfg.fixed_alpha = std::stod(val);
        else if (key == "FIXED_BETA")        cfg.fixed_beta = std::stod(val);
        else if (key == "FIXED_RHO")         cfg.fixed_rho = std::stod(val);
        else if (key == "NUM_ANTS")          cfg.num_ants = std::stoi(val);
        else if (key == "ACO_ITERATIONS")    cfg.aco_iter = std::stoi(val);
        else if (key == "Q_CONST")           cfg.Q_const = std::stod(val);
        else if (key == "TAU_INIT")          cfg.tau_init = std::stod(val);
        else if (key == "SIM_STEPS")         cfg.sim_steps = std::stoi(val);
        else if (key == "TRAFFIC_NOISE")     cfg.traffic_noise = std::stod(val);
        else if (key == "RANDOM_SEED")       cfg.random_seed = std::stoi(val);
        else if (key == "COMBINED_OPT")      cfg.combined_opt = std::stoi(val);
        else if (key == "COMBINED_ITERS")    cfg.combined_iters = std::stoi(val);
        else if (key == "COMBINED_WEIGHT")   cfg.combined_weight = std::stod(val);
        else if (key == "STABILITY_THRESH")  cfg.stability_thresh = std::stod(val);
        else if (key == "DISRUPTION_BLOCK_WEIGHT")   cfg.disruption_block_weight = std::stod(val);
        else if (key == "DISRUPTION_CONGEST_RADIUS") cfg.disruption_congest_radius = std::stoi(val);
        else if (key == "CONGESTION_INCREASE")       cfg.congestion_increase = std::stod(val);
    }
    return cfg;
}

// =============================================================================
//  SECTION 3 -- CSV Loaders
// =============================================================================

std::vector<Edge> load_map(const std::string& path, int& num_nodes) {
    std::vector<Edge> edges;
    std::ifstream f(path);
    if (!f.is_open()) { std::cerr << "[ERR] Cannot open " << path << "\n"; return edges; }
    std::string line;
    std::getline(f, line); // header
    num_nodes = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        Edge e;
        char comma;
        ss >> e.from >> comma >> e.to >> comma >> e.length >> comma >> e.lanes >> comma
           >> e.f_current >> comma >> e.f_in >> comma >> e.f_out >> comma
           >> e.road_class >> comma >> e.width >> comma >> e.road_condition >> comma
           >> e.cat_allowed[0] >> comma >> e.cat_allowed[1] >> comma
           >> e.cat_allowed[2] >> comma >> e.cat_allowed[3] >> comma
           >> e.bus_corridor >> comma >> e.disruption_prior;
        edges.push_back(e);
        num_nodes = std::max(num_nodes, std::max(e.from, e.to) + 1);
    }
    return edges;
}

std::vector<std::pair<int,int>> load_sd_pairs(const std::string& path) {
    std::vector<std::pair<int,int>> pairs;
    std::ifstream f(path);
    if (!f.is_open()) { std::cerr << "[ERR] Cannot open " << path << "\n"; return pairs; }
    std::string line;
    std::getline(f, line); // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        int s, d; char comma;
        ss >> s >> comma >> d;
        pairs.push_back({s, d});
    }
    return pairs;
}

std::vector<DisruptionEvent> load_disruptions(const std::string& path) {
    std::vector<DisruptionEvent> events;
    std::ifstream f(path);
    if (!f.is_open()) { std::cerr << "[WARN] No disruption file: " << path << "\n"; return events; }
    std::string line;
    std::getline(f, line); // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        DisruptionEvent ev;
        char comma;
        ss >> ev.event_id >> comma >> ev.blocked_from >> comma >> ev.blocked_to >> comma;
        std::string et; std::getline(ss, et, ','); ev.event_type = trim(et);
        std::getline(ss, ev.description); ev.description = trim(ev.description);
        events.push_back(ev);
    }
    return events;
}

// =============================================================================
//  SECTION 4 -- Road Condition Factor (Paper Section IV-B) -- PRESERVED
// =============================================================================

// Congestion coefficient (formula 8):
//   Ci(t) = (fi(t-1) + fin_i(t) - fout_i(t)) * L / (li * di)
// Road condition factor (formula 10):
//   Ri(t) = di * (1 + Ci(t))
double congestion_coeff(const Edge& e, double L) {
    double num = (e.f_current + e.f_in - e.f_out) * L;
    double den = e.lanes * e.length;
    if (den < 1e-9) return 0.0;
    double C = num / den;
    return std::max(0.0, C);
}

double road_condition_factor(const Edge& e, double L) {
    return e.length * (1.0 + congestion_coeff(e, L));
}

// Build adjacency lookup: adj[from][to] = edge index
std::vector<std::map<int,int>> build_adjacency(const std::vector<Edge>& edges, int n) {
    std::vector<std::map<int,int>> adj(n);
    for (int i = 0; i < (int)edges.size(); ++i)
        adj[edges[i].from][edges[i].to] = i;
    return adj;
}

// =============================================================================
//  SECTION 5 -- Random Utilities
// =============================================================================

static double rand01() { return (double)rand() / RAND_MAX; }
static double rand_range(double lo, double hi) { return lo + rand01() * (hi - lo); }

// =============================================================================
//  SECTION 6 -- Vehicle Feasibility & Subgraph Filtering
// =============================================================================

// Build a feasible subgraph for a given vehicle category
// Returns a set of edge indices that are accessible
std::set<int> build_feasible_edge_set(
    const std::vector<Edge>& edges, int vehicle_cat, const Config& cfg)
{
    std::set<int> feasible;
    for (int i = 0; i < (int)edges.size(); ++i) {
        const Edge& e = edges[i];
        // Check category access flag
        if (e.cat_allowed[vehicle_cat] == 0) continue;
        // Check minimum width
        if (e.width < cfg.cat_min_width[vehicle_cat]) continue;
        feasible.insert(i);
    }
    return feasible;
}

// Build adjacency only for feasible edges
std::vector<std::map<int,int>> build_feasible_adjacency(
    const std::vector<Edge>& edges, int n, const std::set<int>& feasible_edges)
{
    std::vector<std::map<int,int>> adj(n);
    for (int idx : feasible_edges)
        adj[edges[idx].from][edges[idx].to] = idx;
    return adj;
}

// =============================================================================
//  SECTION 7 -- ACO (Point-to-Point, Road Condition Factor) -- PRESERVED
// =============================================================================

// Transfer probability (formula 1):
//   p_k_ij = [tau_ij]^alpha * [eta_ij]^beta / SUM
// where eta_ij = 1 / R_ij
// Pheromone evaporation (formula 3): tau_ij = (1-rho)*tau_ij + delta_tau
// Pheromone deposit (formula 5): delta_tau_ij = Q / cost_k

ACOResult run_aco(
    const std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& adj,
    int n, int start, int end,
    double alpha, double beta, double rho,
    double Q_const, double tau_init,
    int num_ants, int max_iter, double L)
{
    // Pheromone matrix
    std::vector<std::vector<double>> tau(n, std::vector<double>(n, tau_init));
    ACOResult best;
    best.cost = std::numeric_limits<double>::infinity();
    best.found = false;

    for (int iter = 0; iter < max_iter; ++iter) {
        std::vector<std::vector<double>> delta_tau(n, std::vector<double>(n, 0.0));

        for (int ant = 0; ant < num_ants; ++ant) {
            std::vector<int> path;
            std::vector<bool> visited(n, false);
            path.push_back(start);
            visited[start] = true;
            int cur = start;
            double cost = 0.0;
            bool reached = false;

            for (int step = 0; step < n * 2 && !reached; ++step) {
                std::vector<int> candidates;
                for (auto it = adj[cur].begin(); it != adj[cur].end(); ++it) {
                    int nb = it->first;
                    if (!visited[nb]) candidates.push_back(nb);
                }
                bool end_direct = adj[cur].count(end) > 0;
                if (end_direct && visited[end]) {
                    bool already = false;
                    for (int c : candidates) if (c == end) { already = true; break; }
                    if (!already) candidates.push_back(end);
                }
                if (candidates.empty() && !end_direct) break;

                std::vector<double> probs;
                std::vector<int> choices;
                for (int nb : candidates) {
                    if (adj[cur].count(nb) == 0) continue;
                    int eidx = adj[cur].at(nb);
                    double R = road_condition_factor(edges[eidx], L);
                    double eta = (R > 1e-9) ? 1.0 / R : 1e9;
                    double val = std::pow(tau[cur][nb], alpha) * std::pow(eta, beta);
                    probs.push_back(val);
                    choices.push_back(nb);
                }
                if (choices.empty()) break;

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
                if (cur == end) reached = true;
            }
            if (!reached) continue;

            double deposit = Q_const / cost;
            for (size_t k = 0; k + 1 < path.size(); ++k)
                delta_tau[path[k]][path[k+1]] += deposit;

            if (cost < best.cost) {
                best.cost = cost;
                best.path = path;
                best.found = true;
            }
        }
        // Global pheromone evaporation (formula 3)
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                tau[i][j] = (1.0 - rho) * tau[i][j] + delta_tau[i][j];
    }
    return best;
}

// =============================================================================
//  SECTION 8 -- Pool Generation
// =============================================================================

// Generate a pool of K diverse feasible paths using ACO with pheromone penalties
std::vector<PoolEntry> generate_pool(
    const std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& feasible_adj,
    int n, int start, int end,
    double alpha, double beta, double rho,
    double Q_const, double tau_init,
    int num_ants, int aco_iter, double L, int pool_size)
{
    std::vector<PoolEntry> pool;
    // Penalty pheromone: reduce pheromone on edges used by previously found paths
    std::vector<std::vector<double>> penalty(n, std::vector<double>(n, 0.0));

    for (int p_idx = 0; p_idx < pool_size; ++p_idx) {
        // Create modified pheromone init with penalties
        std::vector<std::vector<double>> tau(n, std::vector<double>(n, tau_init));
        // Apply penalty from previously found paths
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                tau[i][j] = std::max(0.01, tau[i][j] - penalty[i][j]);

        ACOResult res;
        res.cost = std::numeric_limits<double>::infinity();
        res.found = false;

        // Run ACO with penalized pheromones
        for (int iter = 0; iter < aco_iter / 2; ++iter) {
            std::vector<std::vector<double>> delta_tau(n, std::vector<double>(n, 0.0));
            for (int ant = 0; ant < num_ants; ++ant) {
                std::vector<int> path;
                std::vector<bool> visited(n, false);
                path.push_back(start); visited[start] = true;
                int cur = start; double cost = 0.0; bool reached = false;

                for (int step = 0; step < n * 2 && !reached; ++step) {
                    std::vector<int> cands;
                    for (auto it = feasible_adj[cur].begin(); it != feasible_adj[cur].end(); ++it)
                        if (!visited[it->first]) cands.push_back(it->first);
                    bool end_d = feasible_adj[cur].count(end) > 0;
                    if (end_d && visited[end]) {
                        bool al = false;
                        for (int c : cands) if (c == end) { al = true; break; }
                        if (!al) cands.push_back(end);
                    }
                    if (cands.empty() && !end_d) break;

                    std::vector<double> probs; std::vector<int> choices;
                    for (int nb : cands) {
                        if (feasible_adj[cur].count(nb) == 0) continue;
                        int eidx = feasible_adj[cur].at(nb);
                        double R = road_condition_factor(edges[eidx], L);
                        double eta = (R > 1e-9) ? 1.0 / R : 1e9;
                        double val = std::pow(tau[cur][nb], alpha) * std::pow(eta, beta);
                        probs.push_back(val); choices.push_back(nb);
                    }
                    if (choices.empty()) break;
                    double total = 0; for (double pp : probs) total += pp;
                    double r = rand01() * total; double cumul = 0;
                    int chosen = choices.back();
                    for (size_t k = 0; k < choices.size(); ++k) {
                        cumul += probs[k]; if (cumul >= r) { chosen = choices[k]; break; }
                    }
                    int eidx = feasible_adj[cur].at(chosen);
                    cost += road_condition_factor(edges[eidx], L);
                    path.push_back(chosen); visited[chosen] = true; cur = chosen;
                    if (cur == end) reached = true;
                }
                if (!reached) continue;
                double dep = Q_const / cost;
                for (size_t k = 0; k + 1 < path.size(); ++k)
                    delta_tau[path[k]][path[k+1]] += dep;
                if (cost < res.cost) { res.cost = cost; res.path = path; res.found = true; }
            }
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    tau[i][j] = (1.0 - rho) * tau[i][j] + delta_tau[i][j];
        }

        if (res.found) {
            // Check it's not a duplicate
            bool duplicate = false;
            for (const auto& existing : pool)
                if (existing.path == res.path) { duplicate = true; break; }

            if (!duplicate) {
                PoolEntry pe; pe.path = res.path; pe.cost = res.cost; pe.feasible = true;
                pool.push_back(pe);
                // Add penalty on edges of this path for diversity
                for (size_t k = 0; k + 1 < res.path.size(); ++k)
                    penalty[res.path[k]][res.path[k+1]] += tau_init * 0.5;
            }
        }
    }
    // Sort pool by cost (best first)
    std::sort(pool.begin(), pool.end(),
              [](const PoolEntry& a, const PoolEntry& b) { return a.cost < b.cost; });
    return pool;
}

// =============================================================================
//  SECTION 9 -- Traffic Simulation (update fi per step) -- PRESERVED
// =============================================================================

void update_traffic(std::vector<Edge>& edges, double noise) {
    for (auto& e : edges) {
        double net = e.f_current + e.f_in - e.f_out;
        e.f_current = std::max(0.0, net);
        double perturb_in  = 1.0 + rand_range(-noise, noise);
        double perturb_out = 1.0 + rand_range(-noise, noise);
        e.f_in  = std::max(0.0, e.f_in  * perturb_in);
        e.f_out = std::max(0.0, e.f_out * perturb_out);
    }
}

// =============================================================================
//  SECTION 10 -- Helpers
// =============================================================================

static std::string path_to_string(const std::vector<int>& path) {
    std::string s;
    for (size_t i = 0; i < path.size(); ++i) {
        s += std::to_string(path[i]);
        if (i + 1 < path.size()) s += "->";
    }
    return s;
}

static std::string path_to_csv(const std::vector<int>& path) {
    std::string s;
    for (size_t i = 0; i < path.size(); ++i) {
        s += std::to_string(path[i]);
        if (i + 1 < path.size()) s += "-";
    }
    return s;
}

static void print_banner(const std::string& msg) {
    std::string bar(70, '=');
    std::cout << "\n" << bar << "\n  " << msg << "\n" << bar << "\n";
}

static double path_cost_from(
    const std::vector<int>& path, int from_node,
    const std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& adj, double L)
{
    double cost = 0.0; bool counting = false;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (path[i] == from_node) counting = true;
        if (counting && adj[path[i]].count(path[i+1])) {
            int eidx = adj[path[i]].at(path[i+1]);
            cost += road_condition_factor(edges[eidx], L);
        }
    }
    return cost;
}

static double compute_full_path_cost(
    const std::vector<int>& path,
    const std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& adj, double L)
{
    double cost = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (adj[path[i]].count(path[i+1])) {
            int eidx = adj[path[i]].at(path[i+1]);
            cost += road_condition_factor(edges[eidx], L);
        } else {
            return std::numeric_limits<double>::infinity();
        }
    }
    return cost;
}

static bool is_path_feasible(
    const std::vector<int>& path,
    const std::vector<std::map<int,int>>& feasible_adj)
{
    for (size_t i = 0; i + 1 < path.size(); ++i)
        if (feasible_adj[path[i]].count(path[i+1]) == 0) return false;
    return true;
}

static const char* cat_name(int cat) {
    static const char* names[] = {"Truck", "Bus", "Car/Micro", "Rick/Van/Bike"};
    if (cat >= 0 && cat < 4) return names[cat];
    return "Unknown";
}

// Check if an edge is blocked by a disruption
static bool is_edge_blocked(int from, int to,
    const std::vector<DisruptionEvent>& disruptions)
{
    for (const auto& d : disruptions)
        if (d.blocked_from == from && d.blocked_to == to) return true;
    return false;
}

// =============================================================================
//  SECTION 11 -- Step A: Run Single Pair with Vehicle-Aware Pool
//  Flowchart: A1-A12 (independent optimization + steady-state pool)
// =============================================================================

PairResult run_single_pair_vehicle(
    std::vector<Edge> edges,   // BY VALUE: fresh copy
    int n, int src, int dst, int vehicle_cat,
    const Config& cfg,
    std::ofstream& reroute_log, int pair_index)
{
    PairResult result;
    result.source = src; result.destination = dst;
    result.vehicle_cat = vehicle_cat;
    result.replan_count = 0; result.reached = false;

    double L = cfg.avg_vehicle_len;

    // A1: Build feasible subgraph
    std::set<int> feasible_set = build_feasible_edge_set(edges, vehicle_cat, cfg);
    auto feasible_adj = build_feasible_adjacency(edges, n, feasible_set);

    std::cout << "  [A1] Feasible subgraph for " << cat_name(vehicle_cat)
              << ": " << feasible_set.size() << " edges\n";

    if (feasible_set.empty()) {
        std::cerr << "  [WARN] No feasible edges for " << cat_name(vehicle_cat)
                  << " from " << src << " to " << dst << "\n";
        return result;
    }

    // A2: PHASE 1 - Initial pathfinding with fixed alpha, beta, rho
    std::cout << "  [A2] Running improved ACO (fixed a=" << cfg.fixed_alpha
              << " b=" << cfg.fixed_beta << " r=" << cfg.fixed_rho << ") ...\n";

    ACOResult initial = run_aco(edges, feasible_adj, n, src, dst,
        cfg.fixed_alpha, cfg.fixed_beta, cfg.fixed_rho,
        cfg.Q_const, cfg.tau_init, cfg.num_ants, cfg.aco_iter, L);

    if (!initial.found) {
        std::cerr << "  [WARN] ACO found no path " << src << "->" << dst
                  << " for " << cat_name(vehicle_cat) << "\n";
        return result;
    }

    result.initial_path = initial.path;
    result.initial_cost = initial.cost;
    std::cout << "  [A2] besttour: " << path_to_string(initial.path)
              << " cost=" << std::fixed << std::setprecision(2) << initial.cost << "\n";

    // A3: Generate pair-level pool P(s,d,v) of exactly pool_size feasible paths
    std::cout << "  [A3] Generating pool of " << cfg.pool_size << " paths ...\n";
    std::vector<PoolEntry> pool = generate_pool(edges, feasible_adj, n, src, dst,
        cfg.fixed_alpha, cfg.fixed_beta, cfg.fixed_rho,
        cfg.Q_const, cfg.tau_init, cfg.num_ants, cfg.aco_iter, L, cfg.pool_size);

    // Ensure besttour is in pool
    bool bt_in_pool = false;
    for (auto& pe : pool) if (pe.path == initial.path) { bt_in_pool = true; break; }
    if (!bt_in_pool && !pool.empty()) {
        PoolEntry pe; pe.path = initial.path; pe.cost = initial.cost; pe.feasible = true;
        pool.insert(pool.begin(), pe);
        if ((int)pool.size() > cfg.pool_size) pool.pop_back();
    }

    std::cout << "  [A3] Pool generated: " << pool.size() << " paths\n";
    for (int pi = 0; pi < (int)pool.size(); ++pi)
        std::cout << "    P" << pi << ": " << path_to_string(pool[pi].path)
                  << " cost=" << std::setprecision(2) << pool[pi].cost << "\n";

    // A4-A12: Dynamic monitoring loop
    std::cout << "  [A4] Dynamic monitoring over " << cfg.sim_steps << " steps ...\n";

    std::vector<int> best_path = initial.path;
    double best_cost = initial.cost;
    double ar = 0.0;
    int cur_node = src;

    for (int step = 0; step < cfg.sim_steps && cur_node != dst; ++step) {
        // Find position in path
        int pos = -1;
        for (int i = 0; i < (int)best_path.size(); ++i)
            if (best_path[i] == cur_node) { pos = i; break; }
        if (pos < 0 || pos + 1 >= (int)best_path.size()) break;

        int next_node = best_path[pos + 1];
        int eidx = -1;
        if (feasible_adj[cur_node].count(next_node))
            eidx = feasible_adj[cur_node].at(next_node);

        double R_prev = (eidx >= 0) ? road_condition_factor(edges[eidx], L) : 0.0;

        // A4: Dynamic monitoring (t -> t+1)
        update_traffic(edges, cfg.traffic_noise);

        // Update pool feasibility
        for (auto& pe : pool) {
            pe.cost = compute_full_path_cost(pe.path, edges, feasible_adj, L);
            pe.feasible = is_path_feasible(pe.path, feasible_adj) &&
                          pe.cost < std::numeric_limits<double>::infinity();
        }

        // A10: Traverse next segment
        double R_cur = (eidx >= 0) ? road_condition_factor(edges[eidx], L) : 0.0;
        double delta_R = std::fabs(R_cur - R_prev);
        ar += R_cur;
        cur_node = next_node;

        // A5: Trigger condition
        bool trigger = (delta_R > cfg.delta_R_max) || (R_cur > cfg.R_max);
        // Also trigger if current besttour becomes infeasible
        if (!is_path_feasible(best_path, feasible_adj)) trigger = true;
        // Also trigger if a pooled candidate became infeasible
        for (auto& pe : pool) if (!pe.feasible) { trigger = true; break; }

        if (trigger && cur_node != dst) {
            // A7: Re-plan on vehicle-feasible graph / query pool
            ACOResult replan = run_aco(edges, feasible_adj, n, cur_node, dst,
                cfg.fixed_alpha, cfg.fixed_beta, cfg.fixed_rho,
                cfg.Q_const, cfg.tau_init, cfg.num_ants, cfg.aco_iter, L);

            // Also query pool for best feasible candidate from cur_node
            double pool_best_cost = std::numeric_limits<double>::infinity();
            std::vector<int> pool_best_path;
            for (auto& pe : pool) {
                if (!pe.feasible) continue;
                double rem = path_cost_from(pe.path, cur_node, edges, feasible_adj, L);
                if (rem < pool_best_cost) { pool_best_cost = rem; pool_best_path = pe.path; }
            }

            // Choose best between ACO replan and pool query
            std::vector<int> candidate_path;
            double candidate_cost;
            if (replan.found && replan.cost <= pool_best_cost) {
                candidate_path = replan.path; candidate_cost = replan.cost;
            } else if (!pool_best_path.empty()) {
                candidate_path = pool_best_path; candidate_cost = pool_best_cost;
            } else if (replan.found) {
                candidate_path = replan.path; candidate_cost = replan.cost;
            }

            if (!candidate_path.empty()) {
                // A8: Is newtour better and still feasible?
                double remaining_old = path_cost_from(best_path, cur_node, edges, feasible_adj, L);
                if (candidate_cost < remaining_old) {
                    // A9: Accept newtour
                    best_path = candidate_path;
                    best_cost = candidate_cost + ar;
                    cur_node = candidate_path[0];
                    ++result.replan_count;

                    // Update pool ranking
                    std::sort(pool.begin(), pool.end(),
                              [](const PoolEntry& a, const PoolEntry& b) { return a.cost < b.cost; });

                    reroute_log << pair_index << "," << src << "," << dst << ","
                                << vehicle_cat << "," << step << "," << cur_node
                                << ",REROUTE," << std::fixed << std::setprecision(2)
                                << best_cost << "," << ar << ","
                                << path_to_csv(best_path) << "\n";
                } else {
                    reroute_log << pair_index << "," << src << "," << dst << ","
                                << vehicle_cat << "," << step << "," << cur_node
                                << ",TRIGGER_KEPT," << std::fixed << std::setprecision(2)
                                << best_cost << "," << ar << ","
                                << path_to_csv(best_path) << "\n";
                }
            }
        } else {
            // A6: No trigger - proceed on existing besttour
            reroute_log << pair_index << "," << src << "," << dst << ","
                        << vehicle_cat << "," << step << "," << cur_node
                        << ",OK," << std::fixed << std::setprecision(2)
                        << best_cost << "," << ar << ","
                        << path_to_csv(best_path) << "\n";
        }

        // A11: Check endpoint
        if (cur_node == dst) {
            std::cout << "  [A11] Destination " << dst << " reached at step " << step << "\n";
            break;
        }
    }

    // A12: Save pair result
    result.final_path = best_path;
    result.final_cost = ar;
    result.reached = (cur_node == dst);
    result.pool = pool;
    return result;
}

// =============================================================================
//  SECTION 12 -- Step B: Combined Route Optimization + Network-Level Pool
// =============================================================================

void apply_aggregate_traffic(
    std::vector<Edge>& edges,
    const std::vector<std::map<int,int>>& adj,
    const std::vector<PairResult>& results, double weight)
{
    std::map<std::pair<int,int>, int> edge_freq;
    for (const auto& r : results) {
        if (!r.reached) continue;
        for (size_t k = 0; k + 1 < r.final_path.size(); ++k)
            edge_freq[{r.final_path[k], r.final_path[k+1]}]++;
    }
    for (auto& e : edges) {
        auto key = std::make_pair(e.from, e.to);
        auto it = edge_freq.find(key);
        if (it != edge_freq.end() && it->second >= 2) {
            double penalty = weight * (it->second - 1);
            e.f_current += penalty * (e.lanes * e.length / 5.0) * 0.1;
            e.f_in += penalty * 2.0;
        }
    }
}

std::pair<std::vector<PairResult>, std::vector<CombinedIterResult>>
run_combined_optimization(
    const std::vector<Edge>& original_edges,
    int n,
    const std::vector<std::pair<int,int>>& sd_pairs,
    int num_vehicle_cats,
    const Config& cfg)
{
    std::vector<CombinedIterResult> history;
    std::vector<PairResult> best_results, prev_results;
    double prev_avg_cost = 0.0;

    auto full_adj = build_adjacency(original_edges, n);
    std::ofstream combined_rlog("reroute_log_combined.txt");
    combined_rlog << "combined_iter,pair_id,source,destination,vehicle_cat,step,cur_node,"
                  << "event,best_cost,ar,path\n";

    for (int citer = 0; citer < cfg.combined_iters; ++citer) {
        print_banner("COMBINED ITERATION " + std::to_string(citer + 1)
                     + " / " + std::to_string(cfg.combined_iters));

        std::vector<Edge> iter_edges = original_edges;
        if (citer > 0 && !prev_results.empty())
            apply_aggregate_traffic(iter_edges, full_adj, prev_results, cfg.combined_weight);

        std::vector<PairResult> iter_results;
        int pair_counter = 0;

        for (int i = 0; i < (int)sd_pairs.size(); ++i) {
            int src = sd_pairs[i].first, dst = sd_pairs[i].second;
            // Run for each vehicle category that has a feasible path
            for (int vc = 0; vc < num_vehicle_cats; ++vc) {
                std::set<int> fs = build_feasible_edge_set(iter_edges, vc, cfg);
                if (fs.empty()) continue;
                auto fa = build_feasible_adjacency(iter_edges, n, fs);
                // Quick connectivity check
                bool connected = !fa[src].empty();
                if (!connected) continue;

                std::cout << "\n  [B] COMB-ITER " << citer+1 << " PAIR " << i
                          << " (" << src << "->" << dst << ") Cat=" << cat_name(vc) << "\n";

                PairResult r = run_single_pair_vehicle(
                    iter_edges, n, src, dst, vc, cfg, combined_rlog, pair_counter);
                iter_results.push_back(r);
                pair_counter++;
            }
        }

        // B4: Compute iteration statistics
        double total_cost = 0.0; int successful = 0, total_reroutes = 0;
        int route_changes = 0, pool_changes = 0;
        for (int i = 0; i < (int)iter_results.size(); ++i) {
            if (iter_results[i].reached) {
                total_cost += iter_results[i].final_cost; successful++;
            }
            total_reroutes += iter_results[i].replan_count;
            if (citer > 0 && i < (int)prev_results.size()) {
                if (iter_results[i].final_path != prev_results[i].final_path) route_changes++;
                if (iter_results[i].pool.size() != prev_results[i].pool.size()) pool_changes++;
            }
        }
        double avg_cost = (successful > 0) ? total_cost / successful : 0.0;
        double change_pct = (citer > 0 && prev_avg_cost > 0)
                          ? std::fabs(avg_cost - prev_avg_cost) / prev_avg_cost : 1.0;

        CombinedIterResult cir;
        cir.iteration = citer + 1; cir.avg_cost = avg_cost;
        cir.cost_change_pct = change_pct; cir.total_reroutes = total_reroutes;
        cir.route_changes = route_changes; cir.pool_changes = pool_changes;
        history.push_back(cir);

        std::cout << "\n  [B4] ITER " << citer+1 << " avg_cost=" << std::fixed
                  << std::setprecision(2) << avg_cost << " change="
                  << std::setprecision(2) << (change_pct*100) << "%\n";

        prev_results = iter_results; best_results = iter_results;
        prev_avg_cost = avg_cost;

        // B5: Stabilization check
        if (citer > 0 && change_pct < cfg.stability_thresh) {
            std::cout << "  ** STABILIZED at iteration " << citer+1 << " **\n";
            break;
        }
    }
    combined_rlog.close();
    return {best_results, history};
}

// =============================================================================
//  SECTION 13 -- Step C: Online Disruption Response & Case Comparison
// =============================================================================

std::vector<CaseResult> run_disruption_case(
    int case_num,
    const std::vector<Edge>& base_edges,
    int n,
    const std::vector<PairResult>& stable_results,
    const std::vector<DisruptionEvent>& disruptions,
    const Config& cfg,
    std::ofstream& case_log)
{
    std::vector<CaseResult> results;

    for (int ri = 0; ri < (int)stable_results.size(); ++ri) {
        const auto& sr = stable_results[ri];
        if (!sr.reached) continue;

        // C2: Check if this pair is affected
        bool affected = false;
        for (size_t k = 0; k + 1 < sr.final_path.size(); ++k)
            if (is_edge_blocked(sr.final_path[k], sr.final_path[k+1], disruptions))
                { affected = true; break; }
        if (!affected) continue;

        CaseResult cr;
        cr.pair_idx = ri; cr.source = sr.source; cr.destination = sr.destination;
        cr.vehicle_cat = sr.vehicle_cat; cr.case_num = case_num;

        // Create modified edges
        std::vector<Edge> mod_edges = base_edges;

        auto t_start = std::chrono::high_resolution_clock::now();
        int aco_iters_used = 0;

        if (case_num == 1) {
            // CASE 1: Source-based reroute - remove blocked edges, replan from source
            for (auto& e : mod_edges)
                if (is_edge_blocked(e.from, e.to, disruptions))
                    e.f_current = cfg.disruption_block_weight;

            std::set<int> fs = build_feasible_edge_set(mod_edges, sr.vehicle_cat, cfg);
            // Remove blocked edges from feasible set
            for (int i = 0; i < (int)mod_edges.size(); ++i)
                if (is_edge_blocked(mod_edges[i].from, mod_edges[i].to, disruptions))
                    fs.erase(i);
            auto fa = build_feasible_adjacency(mod_edges, n, fs);

            ACOResult replan = run_aco(mod_edges, fa, n, sr.source, sr.destination,
                cfg.fixed_alpha, cfg.fixed_beta, cfg.fixed_rho,
                cfg.Q_const, cfg.tau_init, cfg.num_ants, cfg.aco_iter,
                cfg.avg_vehicle_len);
            aco_iters_used = cfg.aco_iter;

            cr.success = replan.found;
            cr.final_cost = replan.found ? replan.cost : 0.0;
            cr.reroute_path = replan.found ? replan.path : std::vector<int>{};
            cr.reroute_iterations = aco_iters_used;

        } else if (case_num == 2) {
            // CASE 2: Current-position reroute from mid-path
            // Assume vehicle is at the node just before the blockage
            int replan_from = sr.source;
            for (size_t k = 0; k + 1 < sr.final_path.size(); ++k) {
                if (is_edge_blocked(sr.final_path[k], sr.final_path[k+1], disruptions)) {
                    replan_from = sr.final_path[k];
                    break;
                }
            }

            // Increase weights around blocked area
            for (auto& e : mod_edges) {
                if (is_edge_blocked(e.from, e.to, disruptions)) {
                    e.f_current = cfg.disruption_block_weight;
                    continue;
                }
                // Check proximity to blocked edges
                for (const auto& d : disruptions) {
                    if (e.from == d.blocked_from || e.to == d.blocked_from ||
                        e.from == d.blocked_to || e.to == d.blocked_to) {
                        e.f_current *= cfg.congestion_increase;
                        e.f_in *= cfg.congestion_increase;
                    }
                }
            }

            std::set<int> fs = build_feasible_edge_set(mod_edges, sr.vehicle_cat, cfg);
            for (int i = 0; i < (int)mod_edges.size(); ++i)
                if (is_edge_blocked(mod_edges[i].from, mod_edges[i].to, disruptions))
                    fs.erase(i);
            auto fa = build_feasible_adjacency(mod_edges, n, fs);

            ACOResult replan = run_aco(mod_edges, fa, n, replan_from, sr.destination,
                cfg.fixed_alpha, cfg.fixed_beta, cfg.fixed_rho,
                cfg.Q_const, cfg.tau_init, cfg.num_ants, cfg.aco_iter,
                cfg.avg_vehicle_len);
            aco_iters_used = cfg.aco_iter;

            cr.success = replan.found;
            cr.final_cost = replan.found ? replan.cost : 0.0;
            cr.reroute_path = replan.found ? replan.path : std::vector<int>{};
            cr.reroute_iterations = aco_iters_used;

        } else {
            // CASE 3: Pool-based reroute (PROPOSED METHOD)
            // Query precomputed pool, remove infeasible, select best feasible
            std::set<int> fs = build_feasible_edge_set(mod_edges, sr.vehicle_cat, cfg);
            for (int i = 0; i < (int)mod_edges.size(); ++i)
                if (is_edge_blocked(mod_edges[i].from, mod_edges[i].to, disruptions))
                    fs.erase(i);
            auto fa = build_feasible_adjacency(mod_edges, n, fs);

            aco_iters_used = 0; // Pool query, no ACO iterations needed
            cr.success = false;

            for (const auto& pe : sr.pool) {
                // Check if this pooled path avoids all blocked edges
                bool path_ok = true;
                for (size_t k = 0; k + 1 < pe.path.size(); ++k) {
                    if (is_edge_blocked(pe.path[k], pe.path[k+1], disruptions)) {
                        path_ok = false; break;
                    }
                    if (fa[pe.path[k]].count(pe.path[k+1]) == 0) {
                        path_ok = false; break;
                    }
                }
                if (path_ok) {
                    cr.success = true;
                    cr.reroute_path = pe.path;
                    cr.final_cost = compute_full_path_cost(pe.path, mod_edges, fa, cfg.avg_vehicle_len);
                    break; // Take best feasible from sorted pool
                }
            }

            // If no pool path works, do a small repair ACO
            if (!cr.success) {
                ACOResult repair = run_aco(mod_edges, fa, n, sr.source, sr.destination,
                    cfg.fixed_alpha, cfg.fixed_beta, cfg.fixed_rho,
                    cfg.Q_const, cfg.tau_init, cfg.num_ants, cfg.aco_iter / 4,
                    cfg.avg_vehicle_len);
                aco_iters_used = cfg.aco_iter / 4;
                cr.success = repair.found;
                cr.final_cost = repair.found ? repair.cost : 0.0;
                cr.reroute_path = repair.found ? repair.path : std::vector<int>{};
            }
            cr.reroute_iterations = aco_iters_used;
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        cr.reroute_time_ms = elapsed_ms;
        cr.energy_proxy = elapsed_ms * 0.001; // Simplified energy proxy (clearly labeled)

        results.push_back(cr);

        case_log << "Case" << case_num << ",P" << ri << "," << sr.source << "->"
                 << sr.destination << "," << cat_name(sr.vehicle_cat) << ","
                 << (cr.success ? "YES" : "NO") << ","
                 << std::fixed << std::setprecision(2) << cr.final_cost << ","
                 << cr.reroute_iterations << ","
                 << std::setprecision(4) << cr.reroute_time_ms << "ms,"
                 << path_to_string(cr.reroute_path) << "\n";
    }
    return results;
}

// =============================================================================
//  SECTION 14 -- Output File Writers
// =============================================================================

void save_pair_stable_routes(const std::vector<PairResult>& results) {
    std::ofstream f("pair_stable_routes.csv");
    f << "pair_id,source,destination,vehicle_cat,initial_path,initial_cost,"
      << "final_path,final_cost,reroutes,reached\n";
    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        f << i << "," << r.source << "," << r.destination << ","
          << cat_name(r.vehicle_cat) << ","
          << path_to_csv(r.initial_path) << ","
          << std::fixed << std::setprecision(2) << r.initial_cost << ","
          << path_to_csv(r.final_path) << ","
          << r.final_cost << "," << r.replan_count << ","
          << (r.reached ? "YES" : "NO") << "\n";
    }
    f.close();
}

void save_pair_pools(const std::vector<PairResult>& results) {
    std::ofstream f("pair_pools.json");
    f << "[\n";
    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        f << "  {\"pair_id\":" << i << ",\"source\":" << r.source
          << ",\"destination\":" << r.destination
          << ",\"vehicle_cat\":\"" << cat_name(r.vehicle_cat) << "\""
          << ",\"pool\":[\n";
        for (int j = 0; j < (int)r.pool.size(); ++j) {
            f << "    {\"path\":\"" << path_to_string(r.pool[j].path) << "\""
              << ",\"cost\":" << std::fixed << std::setprecision(2) << r.pool[j].cost
              << ",\"feasible\":" << (r.pool[j].feasible ? "true" : "false") << "}";
            if (j + 1 < (int)r.pool.size()) f << ",";
            f << "\n";
        }
        f << "  ]}";
        if (i + 1 < (int)results.size()) f << ",";
        f << "\n";
    }
    f << "]\n";
    f.close();
}

void save_pair_metadata(const std::vector<PairResult>& results) {
    std::ofstream f("pair_metadata.csv");
    f << "pair_id,source,destination,vehicle_cat,pool_size,replan_count,reached,final_cost\n";
    for (int i = 0; i < (int)results.size(); ++i) {
        const auto& r = results[i];
        f << i << "," << r.source << "," << r.destination << ","
          << cat_name(r.vehicle_cat) << "," << r.pool.size() << ","
          << r.replan_count << "," << (r.reached ? "YES" : "NO") << ","
          << std::fixed << std::setprecision(2) << r.final_cost << "\n";
    }
    f.close();
}

void save_case_results(const std::vector<CaseResult>& results, int case_num) {
    std::string fname = "case" + std::to_string(case_num) + "_results.csv";
    std::ofstream f(fname);
    f << "pair_idx,source,destination,vehicle_cat,reroute_iterations,"
      << "reroute_time_ms,energy_proxy,success,final_cost,reroute_path\n";
    for (const auto& r : results) {
        f << r.pair_idx << "," << r.source << "," << r.destination << ","
          << cat_name(r.vehicle_cat) << "," << r.reroute_iterations << ","
          << std::fixed << std::setprecision(4) << r.reroute_time_ms << ","
          << r.energy_proxy << ","
          << (r.success ? "YES" : "NO") << ","
          << std::setprecision(2) << r.final_cost << ","
          << path_to_csv(r.reroute_path) << "\n";
    }
    f.close();
}

void save_case_comparison(
    const std::vector<CaseResult>& c1,
    const std::vector<CaseResult>& c2,
    const std::vector<CaseResult>& c3)
{
    std::ofstream f("case_comparison_summary.csv");
    f << "case,total_affected,successful_reroutes,avg_iterations,"
      << "avg_reroute_time_ms,avg_energy_proxy,avg_final_cost\n";

    auto summarize = [&](const std::vector<CaseResult>& results, int cn) {
        int total = results.size(), success = 0;
        double sum_iter = 0, sum_time = 0, sum_energy = 0, sum_cost = 0;
        for (const auto& r : results) {
            if (r.success) { success++; sum_cost += r.final_cost; }
            sum_iter += r.reroute_iterations;
            sum_time += r.reroute_time_ms;
            sum_energy += r.energy_proxy;
        }
        double n = std::max(1, total);
        f << cn << "," << total << "," << success << ","
          << std::fixed << std::setprecision(2) << sum_iter/n << ","
          << std::setprecision(4) << sum_time/n << ","
          << sum_energy/n << ","
          << std::setprecision(2) << (success > 0 ? sum_cost/success : 0) << "\n";
    };

    summarize(c1, 1);
    summarize(c2, 2);
    summarize(c3, 3);
    f.close();
}

void save_network_pool_analysis(
    const std::vector<PairResult>& results,
    const std::vector<Edge>& edges, int n)
{
    std::ofstream f("network_pool_analysis.txt");
    f << "================================================================\n"
      << "  NETWORK-LEVEL POOL ANALYSIS\n"
      << "================================================================\n\n";

    // Edge usage
    std::map<std::pair<int,int>, int> edge_freq;
    for (const auto& r : results) {
        if (!r.reached) continue;
        for (size_t k = 0; k + 1 < r.final_path.size(); ++k)
            edge_freq[{r.final_path[k], r.final_path[k+1]}]++;
    }
    f << "1. EDGE USAGE FREQUENCY\n";
    std::vector<std::pair<std::pair<int,int>, int>> sorted_edges(edge_freq.begin(), edge_freq.end());
    std::sort(sorted_edges.begin(), sorted_edges.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    for (auto& se : sorted_edges)
        f << "   " << se.first.first << " -> " << se.first.second
          << "  freq=" << se.second << "\n";

    // Pool diversity
    f << "\n2. POOL DIVERSITY PER PAIR\n";
    int total_pool_paths = 0, total_unique = 0;
    for (int i = 0; i < (int)results.size(); ++i) {
        f << "   P" << i << " (" << results[i].source << "->" << results[i].destination
          << " " << cat_name(results[i].vehicle_cat) << "): "
          << results[i].pool.size() << " pool paths\n";
        total_pool_paths += results[i].pool.size();
    }
    f << "\n   Total pool paths across all pairs: " << total_pool_paths << "\n";

    // Vehicle category breakdown
    f << "\n3. VEHICLE CATEGORY BREAKDOWN\n";
    int cat_counts[4] = {0,0,0,0};
    int cat_reached[4] = {0,0,0,0};
    for (const auto& r : results) {
        if (r.vehicle_cat >= 0 && r.vehicle_cat < 4) {
            cat_counts[r.vehicle_cat]++;
            if (r.reached) cat_reached[r.vehicle_cat]++;
        }
    }
    for (int c = 0; c < 4; ++c)
        f << "   " << cat_name(c) << ": " << cat_reached[c] << "/" << cat_counts[c]
          << " reached\n";

    f.close();
}

// =============================================================================
//  SECTION 15 -- MAIN
// =============================================================================

int main() {
    print_banner("Vehicle-Aware Pool-Based Rerouting Framework for Dhaka");
    std::cout << "  Extended from: Wu et al. 2020 Improved ACO\n"
              << "  Follows: Detailed Combined Supervisor Flowchart\n";

    // -- Load config --
    Config cfg = load_config("config.txt");
    if (cfg.random_seed != 0) srand(cfg.random_seed);
    else srand(static_cast<unsigned>(time(nullptr)));

    std::cout << "\n[CFG] map=" << cfg.map_file << "  pairs=" << cfg.pairs_file
              << "  disruption=" << cfg.disruption_file << "\n";

    // -- Load map (extended schema) --
    int n = 0;
    std::vector<Edge> original_edges = load_map(cfg.map_file, n);
    auto full_adj = build_adjacency(original_edges, n);
    std::cout << "[MAP] " << n << " nodes, " << original_edges.size() << " edges\n";

    // -- Compute initial R_i(t) --
    std::cout << "\n[INIT] Road Condition Factors + Extended Attributes:\n";
    std::cout << "  " << std::setw(8) << "Edge" << std::setw(8) << "Len"
              << std::setw(5) << "RC" << std::setw(6) << "W"
              << std::setw(8) << "R(t)" << std::setw(5) << "C1"
              << std::setw(5) << "C2" << std::setw(5) << "C3"
              << std::setw(5) << "C4" << std::setw(5) << "BC"
              << std::setw(6) << "DP" << "\n";
    for (const auto& e : original_edges) {
        double R = road_condition_factor(e, cfg.avg_vehicle_len);
        std::cout << "  " << std::setw(3) << e.from << "->" << std::setw(2) << e.to
                  << std::setw(8) << std::fixed << std::setprecision(1) << e.length
                  << std::setw(5) << e.road_class
                  << std::setw(6) << std::setprecision(1) << e.width
                  << std::setw(8) << std::setprecision(2) << R
                  << std::setw(5) << e.cat_allowed[0]
                  << std::setw(5) << e.cat_allowed[1]
                  << std::setw(5) << e.cat_allowed[2]
                  << std::setw(5) << e.cat_allowed[3]
                  << std::setw(5) << e.bus_corridor
                  << std::setw(6) << std::setprecision(2) << e.disruption_prior << "\n";
    }

    // -- Load S-D pairs --
    auto sd_pairs = load_sd_pairs(cfg.pairs_file);
    std::cout << "\n[PAIRS] " << sd_pairs.size() << " pairs loaded.\n";

    // -- Validate pairs --
    for (int i = 0; i < (int)sd_pairs.size(); ++i) {
        int s = sd_pairs[i].first, d = sd_pairs[i].second;
        if (s >= n || d >= n || s < 0 || d < 0 || s == d) {
            std::cerr << "[ERR] Invalid pair P" << i << "\n"; return 1;
        }
    }

    // -- Load disruption events --
    auto disruptions = load_disruptions(cfg.disruption_file);
    std::cout << "[DISRUPTIONS] " << disruptions.size() << " events loaded.\n";

    // =========================================================================
    //  STEP A: Independent Optimization + Pool Generation
    // =========================================================================
    print_banner("STEP A: INDEPENDENT OPTIMIZATION + STEADY-STATE POOL");

    std::ofstream reroute_log("reroute_log.txt");
    reroute_log << "pair_id,source,destination,vehicle_cat,step,cur_node,event,"
                << "best_cost,ar,path\n";

    std::vector<PairResult> all_results;
    int pair_counter = 0;

    for (int i = 0; i < (int)sd_pairs.size(); ++i) {
        int src = sd_pairs[i].first, dst = sd_pairs[i].second;

        for (int vc = 0; vc < cfg.vehicle_categories; ++vc) {
            // Check if this category has any feasible edges at all
            std::set<int> fs = build_feasible_edge_set(original_edges, vc, cfg);
            if (fs.empty()) continue;

            print_banner("PAIR " + std::to_string(i) + " (" + std::to_string(src)
                         + " -> " + std::to_string(dst) + ") Cat=" + cat_name(vc));

            PairResult r = run_single_pair_vehicle(
                original_edges, n, src, dst, vc, cfg, reroute_log, pair_counter);

            all_results.push_back(r);
            pair_counter++;

            std::cout << "  Result: reached=" << (r.reached ? "YES" : "NO")
                      << " cost=" << std::fixed << std::setprecision(2) << r.final_cost
                      << " replans=" << r.replan_count
                      << " pool=" << r.pool.size() << "\n";
        }
    }
    reroute_log.close();

    // Save Step A outputs
    save_pair_stable_routes(all_results);
    save_pair_pools(all_results);
    save_pair_metadata(all_results);
    std::cout << "\n[A12] Saved: pair_stable_routes.csv, pair_pools.json, pair_metadata.csv\n";

    // =========================================================================
    //  STEP B: Combined Route Optimization (if enabled)
    // =========================================================================
    std::vector<PairResult> combined_results;
    std::vector<CombinedIterResult> combined_history;

    if (cfg.combined_opt) {
        print_banner("STEP B: COMBINED ROUTE OPTIMIZATION + NETWORK-LEVEL POOL");

        auto comb_pair = run_combined_optimization(
            original_edges, n, sd_pairs, cfg.vehicle_categories, cfg);
        combined_results = comb_pair.first;
        combined_history = comb_pair.second;

        // Use combined results for disruption response if available
        if (!combined_results.empty()) {
            save_pair_stable_routes(combined_results); // overwrite with combined
            save_pair_pools(combined_results);
            save_pair_metadata(combined_results);
        }

        // B6: Save combined report
        save_network_pool_analysis(
            combined_results.empty() ? all_results : combined_results,
            original_edges, n);
        std::cout << "[B6] Saved: network_pool_analysis.txt\n";
    } else {
        save_network_pool_analysis(all_results, original_edges, n);
    }

    // =========================================================================
    //  STEP C: Online Disruption Response and Case Comparison
    // =========================================================================
    print_banner("STEP C: ONLINE DISRUPTION RESPONSE AND CASE COMPARISON");

    const auto& stable_results = combined_results.empty() ? all_results : combined_results;

    std::cout << "  Processing " << disruptions.size() << " disruption events ...\n";
    for (const auto& d : disruptions)
        std::cout << "  Event " << d.event_id << ": " << d.blocked_from << "->"
                  << d.blocked_to << " (" << d.event_type << ")\n";

    // C3: Run all three cases
    std::ofstream log1("reroute_log_case1.txt"); log1 << "case,pair,route,cat,success,cost,iters,time,path\n";
    std::ofstream log2("reroute_log_case2.txt"); log2 << "case,pair,route,cat,success,cost,iters,time,path\n";
    std::ofstream log3("reroute_log_case3.txt"); log3 << "case,pair,route,cat,success,cost,iters,time,path\n";

    print_banner("CASE 1: SOURCE-BASED REROUTE");
    auto case1_results = run_disruption_case(1, original_edges, n, stable_results, disruptions, cfg, log1);

    print_banner("CASE 2: CURRENT-POSITION REROUTE");
    auto case2_results = run_disruption_case(2, original_edges, n, stable_results, disruptions, cfg, log2);

    print_banner("CASE 3: POOL-BASED REROUTE (PROPOSED METHOD)");
    auto case3_results = run_disruption_case(3, original_edges, n, stable_results, disruptions, cfg, log3);

    log1.close(); log2.close(); log3.close();

    // C4: Save case results and comparison
    save_case_results(case1_results, 1);
    save_case_results(case2_results, 2);
    save_case_results(case3_results, 3);
    save_case_comparison(case1_results, case2_results, case3_results);

    // C5: Print hypothesis
    std::cout << "\n  ================================================================\n"
              << "  RESEARCH HYPOTHESIS (to be validated experimentally):\n"
              << "  Case 3 (pool-based reroute) is expected to achieve the smallest\n"
              << "  online reroute iterations, reroute time, and computation power\n"
              << "  while preserving category-wise feasible rerouting.\n"
              << "  ================================================================\n";

    // Print comparison summary
    std::cout << "\n  CASE COMPARISON SUMMARY:\n";
    auto print_case_summary = [](const std::vector<CaseResult>& res, int cn) {
        int total = res.size(), success = 0;
        double sum_iter = 0, sum_time = 0;
        for (const auto& r : res) {
            if (r.success) success++;
            sum_iter += r.reroute_iterations;
            sum_time += r.reroute_time_ms;
        }
        double nn = std::max(1, total);
        std::cout << "  Case " << cn << ": affected=" << total
                  << " success=" << success
                  << " avg_iters=" << std::fixed << std::setprecision(1) << sum_iter/nn
                  << " avg_time=" << std::setprecision(3) << sum_time/nn << "ms\n";
    };
    print_case_summary(case1_results, 1);
    print_case_summary(case2_results, 2);
    print_case_summary(case3_results, 3);

    // -- Final output listing --
    print_banner("OUTPUT FILES GENERATED");
    std::cout << "  pair_stable_routes.csv\n"
              << "  pair_pools.json\n"
              << "  pair_metadata.csv\n"
              << "  network_pool_analysis.txt\n"
              << "  case1_results.csv\n"
              << "  case2_results.csv\n"
              << "  case3_results.csv\n"
              << "  case_comparison_summary.csv\n"
              << "  reroute_log.txt\n"
              << "  reroute_log_case1.txt\n"
              << "  reroute_log_case2.txt\n"
              << "  reroute_log_case3.txt\n"
              << "  reroute_log_combined.txt\n";

    print_banner("Done. Vehicle-aware pool-based rerouting framework complete.");
    return 0;
}
