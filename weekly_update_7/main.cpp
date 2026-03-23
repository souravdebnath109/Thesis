#include <bits/stdc++.h>
using namespace std;

/*
  Dynamic Path Planning (Paper-style) demo with 5 nodes.

  - tour_init: initial route cost using PSO->optimized ACO params, weights = R(t=0)
  - tour_static: follow same initial route, but accumulate R(t) as traffic changes
  - tour_aco: dynamic ACO with distance-only weights (d)
  - tour_opt: improved dynamic ACO with road condition factor weights R(t), replan if big change

  NOTE:
  - This is a minimal reproducible implementation for 5 nodes (not production).
  - Traffic is simulated by default. Google Maps integration is OPTIONAL (see stub).
*/

static std::mt19937 rng(123456);

// --------------------------- Google Maps (Optional) ---------------------------
// If you want to pull real distances/durations from Google Maps Distance Matrix:
// You’d normally use libcurl + a JSON parser.
// To keep this file dependency-free, we provide a stub and a manual override.
//
// If you want real API integration, I can give you a version using libcurl + nlohmann/json.

struct GoogleEdgeData {
    double distance_m = 0.0;  // meters
    double duration_s = 0.0;  // seconds (can represent traffic)
    bool ok = false;
};

GoogleEdgeData fetchDistanceMatrixStub(
    const string& apiKey,
    double origLat, double origLng,
    double destLat, double destLng
) {
    // STUB (no network here). Return ok=false so the code falls back to mock.
    (void)apiKey; (void)origLat; (void)origLng; (void)destLat; (void)destLng;
    return {};
}

// ------------------------------ Graph Model ----------------------------------

struct Edge {
    int to;
    int id;          // unique edge id
    double d;        // distance (meters)
    int lanes;       // number of lanes
    // Traffic state for congestion coefficient:
    double f_prev;   // vehicles on edge at t-1
    double f_in;     // inbound flow at time t
    double f_out;    // outbound flow at time t
};

struct RoadState {
    // Road condition factor R(t)
    double R;
    // Congestion coefficient C(t)
    double C;
};

struct Graph {
    int n;
    vector<vector<Edge>> adj;
    // Store base edge data by edgeId (for convenience):
    vector<pair<int,int>> edgeEndpoints; // (u, idx-in-adj[u]) for each edgeId

    Graph(int n=0): n(n), adj(n) {}

    void addUndirectedEdge(int u, int v, double dist_m, int lanes, int edgeId) {
        Edge e1{v, edgeId, dist_m, lanes, 0, 0, 0};
        Edge e2{u, edgeId, dist_m, lanes, 0, 0, 0};
        int idx1 = (int)adj[u].size();
        int idx2 = (int)adj[v].size();
        adj[u].push_back(e1);
        adj[v].push_back(e2);
        // Keep one endpoint mapping (we'll update both directions by scanning)
        if ((int)edgeEndpoints.size() <= edgeId) edgeEndpoints.resize(edgeId+1);
        edgeEndpoints[edgeId] = {u, idx1};
        (void)idx2;
    }
};

// --------------------------- Traffic / Road Factor ----------------------------

static constexpr double VEHICLE_LEN_L = 4.5; // average vehicle length (meters)

// Paper-style congestion coefficient (simplified):
// C(t) = ((f_prev + f_in - f_out) * L) / (lanes * d) if (f_prev + f_in > f_out) else 0
// R(t) = d * (1 + C(t))
RoadState computeRoadState(const Edge& e) {
    RoadState rs;
    double numerator = (e.f_prev + e.f_in - e.f_out);
    if (e.f_prev + e.f_in > e.f_out && numerator > 0) {
        rs.C = (numerator * VEHICLE_LEN_L) / ( (double)e.lanes * e.d );
    } else {
        rs.C = 0.0;
    }
    rs.R = e.d * (1.0 + rs.C);
    return rs;
}

// Simulate traffic update each time step:
// - random flows, with constraint inbound/outbound <= 1 (like paper idea)
void simulateTraffic(Graph& g, int t) {
    (void)t;

    // Higher inflow = more congestion
    std::uniform_real_distribution<double> baseFlow(30.0, 80.0);

    // outbound smaller -> congestion increases
    std::uniform_real_distribution<double> outRatio(0.1, 0.6);

    for (int u = 0; u < g.n; ++u) {
        for (auto &e : g.adj[u]) {
            double fin = baseFlow(rng);
            double ratio = outRatio(rng);

            double fout = fin * ratio; // fout is smaller than fin -> congestion

            e.f_prev = max(0.0, e.f_prev + fin - fout);
            e.f_in = fin;
            e.f_out = fout;
        }
    }
}

// ----------------------------- ACO Implementation ----------------------------

// Build a path using ACO from start to end on a small graph.
// We do NOT solve TSP; we solve point-to-point path planning.
// - If useRoadFactor = false: weight = distance d
// - If useRoadFactor = true:  weight = R(t) computed from current traffic state
struct ACOParams {
    double alpha; // pheromone influence
    double beta;  // heuristic influence
    double rho;   // evaporation
};

struct ACOResult {
    vector<int> path;
    double cost;
};

double edgeCost(const Graph& g, int u, int idx, bool useRoadFactor) {
    const Edge& e = g.adj[u][idx];
    if (!useRoadFactor) return e.d;
    RoadState rs = computeRoadState(e);
    return rs.R;
}

ACOResult runACO_PointToPoint(
    const Graph& g,
    int start, int goal,
    const ACOParams& p,
    bool useRoadFactor,
    int ants = 10,
    int iterations = 80
) {
    int n = g.n;

    // Pheromone matrix for directed edges: tau[u][k] corresponds to edge index k in adj[u]
    vector<vector<double>> tau(n);
    for (int u=0; u<n; ++u) {
        tau[u].assign(g.adj[u].size(), 1.0); // initial pheromone
    }

    auto heuristic = [&](int u, int edgeIdx) {
        double c = edgeCost(g, u, edgeIdx, useRoadFactor);
        return 1.0 / max(1e-9, c);
    };

    ACOResult best;
    best.cost = 1e100;

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (int it=0; it<iterations; ++it) {
        vector<vector<double>> delta(n);
        for (int u=0; u<n; ++u) delta[u].assign(g.adj[u].size(), 0.0);

        for (int k=0; k<ants; ++k) {
            int cur = start;
            vector<int> path;
            path.push_back(cur);
            double costSum = 0.0;

            // To avoid infinite loops, keep visited set and step cap
            vector<int> visited(n, 0);
            visited[cur] = 1;
            int stepCap = 2*n + 10;

            while (cur != goal && stepCap-- > 0) {
                // Compute transition probabilities among neighbors
                double denom = 0.0;
                vector<double> prob(g.adj[cur].size(), 0.0);

                for (int ei=0; ei<(int)g.adj[cur].size(); ++ei) {
                    int nxt = g.adj[cur][ei].to;
                    // allow revisits (because point-to-point), but discourage loops:
                    double loopPenalty = visited[nxt] ? 0.2 : 1.0;

                    double val = pow(tau[cur][ei], p.alpha) * pow(heuristic(cur, ei), p.beta) * loopPenalty;
                    prob[ei] = val;
                    denom += val;
                }

                if (denom <= 1e-12) break;

                // Roulette wheel selection
                double r = uni(rng) * denom;
                int chosen = -1;
                double acc = 0.0;
                for (int ei=0; ei<(int)prob.size(); ++ei) {
                    acc += prob[ei];
                    if (acc >= r) { chosen = ei; break; }
                }
                if (chosen < 0) chosen = (int)prob.size() - 1;

                int nxt = g.adj[cur][chosen].to;
                double c = edgeCost(g, cur, chosen, useRoadFactor);

                costSum += c;
                cur = nxt;
                path.push_back(cur);
                visited[cur]++;

                if (path.size() > (size_t)(3*n + 20)) break;
            }

            if (cur == goal) {
                // deposit pheromone proportional to quality
                double Q = 100.0;
                double deposit = Q / max(1e-9, costSum);

                // Add pheromone along edges in the found path
                for (int i=0; i+1<(int)path.size(); ++i) {
                    int u = path[i], v = path[i+1];
                    // find edge index u->v
                    for (int ei=0; ei<(int)g.adj[u].size(); ++ei) {
                        if (g.adj[u][ei].to == v) {
                            delta[u][ei] += deposit;
                            break;
                        }
                    }
                }

                if (costSum < best.cost) {
                    best.cost = costSum;
                    best.path = path;
                }
            }
        }

        // evaporate and update pheromone
        for (int u=0; u<n; ++u) {
            for (int ei=0; ei<(int)tau[u].size(); ++ei) {
                tau[u][ei] = (1.0 - p.rho) * tau[u][ei] + delta[u][ei];
                tau[u][ei] = max(1e-9, tau[u][ei]);
            }
        }
    }

    if (best.cost >= 1e99) {
        // fallback: no path found
        best.path = {start};
        best.cost = 1e100;
    }
    return best;
}

// ----------------------------- PSO (alpha,beta,rho) --------------------------
//
// PSO optimizes ACO parameters by minimizing initial path cost (tour_init) from a sampled set of pairs.
// For a 5-node demo, we keep it simple and optimize to one start/end per run.
// You can generalize by averaging over multiple (start,end) pairs.

struct Particle {
    double a, b, r;
    double va, vb, vr;
    double bestA, bestB, bestR;
    double bestFit;
};

double clamp(double x, double lo, double hi) { return max(lo, min(hi, x)); }

ACOParams runPSO_for_ACOParams(
    const Graph& g,
    int start, int goal,
    bool useRoadFactorForInit, // paper uses R(0) in PSO step
    int swarmSize = 25,
    int iters = 35
) {
    // Parameter ranges (paper mentions ranges; we use common ACO ranges)
    // alpha in [0.5, 1.5], beta in [1, 5], rho in [0.5, 1.0]
    double aLo=0.5, aHi=1.5;
    double bLo=1.0, bHi=5.0;
    double rLo=0.5, rHi=1.0;

    std::uniform_real_distribution<double> ua(aLo, aHi), ub(bLo, bHi), ur(rLo, rHi);
    std::uniform_real_distribution<double> uv(-0.2, 0.2);

    auto fitness = [&](double a, double b, double r) {
        ACOParams p{a,b,r};
        auto res = runACO_PointToPoint(g, start, goal, p, useRoadFactorForInit, 10, 60);
        return res.cost;
    };

    vector<Particle> swarm(swarmSize);
    double gBestFit = 1e100;
    ACOParams gBest{1.0, 2.5, 0.8};

    for (auto &pt : swarm) {
        pt.a = ua(rng); pt.b = ub(rng); pt.r = ur(rng);
        pt.va = uv(rng); pt.vb = uv(rng); pt.vr = uv(rng);
        pt.bestA = pt.a; pt.bestB = pt.b; pt.bestR = pt.r;
        pt.bestFit = fitness(pt.a, pt.b, pt.r);
        if (pt.bestFit < gBestFit) {
            gBestFit = pt.bestFit;
            gBest = {pt.bestA, pt.bestB, pt.bestR};
        }
    }

    // PSO hyperparams
    double w = 0.7;
    double c1 = 1.4;
    double c2 = 1.4;
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    for (int it=0; it<iters; ++it) {
        for (auto &pt : swarm) {
            double r1=u01(rng), r2=u01(rng);
            pt.va = w*pt.va + c1*r1*(pt.bestA - pt.a) + c2*r2*(gBest.alpha - pt.a);
            pt.vb = w*pt.vb + c1*r1*(pt.bestB - pt.b) + c2*r2*(gBest.beta  - pt.b);
            pt.vr = w*pt.vr + c1*r1*(pt.bestR - pt.r) + c2*r2*(gBest.rho   - pt.r);

            pt.a = clamp(pt.a + pt.va, aLo, aHi);
            pt.b = clamp(pt.b + pt.vb, bLo, bHi);
            pt.r = clamp(pt.r + pt.vr, rLo, rHi);

            double f = fitness(pt.a, pt.b, pt.r);
            if (f < pt.bestFit) {
                pt.bestFit = f;
                pt.bestA = pt.a; pt.bestB = pt.b; pt.bestR = pt.r;
                if (f < gBestFit) {
                    gBestFit = f;
                    gBest = {pt.bestA, pt.bestB, pt.bestR};
                }
            }
        }
    }

    return gBest;
}

// --------------------- Experiment Measures (Paper-style) ----------------------

double pathCostOnCurrentTraffic(const Graph& g, const vector<int>& path, bool useRoadFactor) {
    double sum=0.0;
    for (int i=0; i+1<(int)path.size(); ++i) {
        int u=path[i], v=path[i+1];
        bool found=false;
        for (int ei=0; ei<(int)g.adj[u].size(); ++ei) {
            if (g.adj[u][ei].to == v) {
                sum += edgeCost(g, u, ei, useRoadFactor);
                found=true;
                break;
            }
        }
        if (!found) return 1e100;
    }
    return sum;
}

bool isClearPath(const Graph& g, const vector<int>& path) {
    // "clear path": all edges have Ci(t) in [0,4) (paper says if entire planning path is clear -> invalid)
    // We'll treat "clear" as all C < 4.
    for (int i=0; i+1<(int)path.size(); ++i) {
        int u=path[i], v=path[i+1];
        for (auto &e : g.adj[u]) {
            if (e.to == v) {
                auto rs = computeRoadState(e);
                if (!(rs.C >= 0.0 && rs.C < 4.0)) return false;
                break;
            }
        }
    }
    return true;
}

// Improved dynamic ACO idea: if change is large, replan (paper uses thresholds Rmax and dRmax).
// Here: replan if any edge along current remaining path has R(t) increased by > dRmax or exceeds Rmax.
bool shouldReplan(const Graph& g, const vector<int>& remainingPath, double Rmax, double dRmax, double& worstDeltaR) {
    worstDeltaR = 0.0;
    for (int i=0; i+1<(int)remainingPath.size(); ++i) {
        int u=remainingPath[i], v=remainingPath[i+1];
        for (auto &e : g.adj[u]) {
            if (e.to == v) {
                double Rnow = computeRoadState(e).R;
                // Approx delta via f_prev evolution: here we can't access "previous R" directly.
                // So we interpret "delta" via current congestion C (proxy). For a demo, we use:
                double deltaProxy = computeRoadState(e).C * e.d; // scaled
                worstDeltaR = max(worstDeltaR, deltaProxy);
                if (Rnow > Rmax || deltaProxy > dRmax) return true;
                break;
            }
        }
    }
    return false;
}

// ------------------------------ Demo Setup -----------------------------------
//
// 5 nodes (you can replace with real Dhaka intersections later):
// 0: Farmgate
// 1: Shahbagh
// 2: Gulistan
// 3: Mohakhali
// 4: Motijheel
//
// Distances are mock meters; lanes are mock.

Graph buildDhaka5NodeGraph_Mock() {
    Graph g(5);

    // edgeId increments for uniqueness, but we keep them simple.
    int eid = 0;
    g.addUndirectedEdge(0, 1, 2200, 3, eid++); // Farmgate <-> Shahbagh
    g.addUndirectedEdge(1, 2, 3000, 2, eid++); // Shahbagh <-> Gulistan
    g.addUndirectedEdge(2, 4, 2500, 3, eid++); // Gulistan <-> Motijheel
    g.addUndirectedEdge(0, 3, 1800, 3, eid++); // Farmgate <-> Mohakhali
    g.addUndirectedEdge(3, 4, 4200, 2, eid++); // Mohakhali <-> Motijheel
    g.addUndirectedEdge(1, 3, 2000, 2, eid++); // Shahbagh <-> Mohakhali

    // Initialize f_prev to something
    for (int u=0; u<g.n; ++u) {
        for (auto &e : g.adj[u]) {
            e.f_prev = 20.0;
        }
    }
    return g;
}

// ------------------------------ Main Experiment -------------------------------

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    Graph g = buildDhaka5NodeGraph_Mock();

    // Experiment config
    int nTrials = 50;          // like "n" in paper, but smaller
    int ants = 10;
    int acoIters = 80;

    double sum_init=0, sum_static=0, sum_aco=0, sum_opt=0;
    int validCount = 0;

    // Thresholds for replanning (demo values; tune them)
    double Rmax = 8000.0;      // if an edge's R gets too large
    double dRmax = 1500.0;     // if change proxy gets too large

    std::uniform_int_distribution<int> pickNode(0, g.n-1);

    for (int trial=0; trial<nTrials; ++trial) {
        // Reset / simulate initial traffic at t=0
        simulateTraffic(g, 0);

        int start = pickNode(rng);
        int goal  = pickNode(rng);
        while (goal == start) goal = pickNode(rng);

        // Step 1: PSO to optimize ACO parameters for initial planning using R(0)
        ACOParams bestParams = runPSO_for_ACOParams(g, start, goal, /*useRoadFactorForInit=*/true);

        // Step 2: tour_init via ACO using road factor at t=0 with PSO-optimized params
        auto initRes = runACO_PointToPoint(g, start, goal, bestParams, /*useRoadFactor=*/true, ants, acoIters);
        double tour_init = initRes.cost;
        vector<int> initPath = initRes.path;

        if (tour_init >= 1e99 || initPath.size() < 2) continue;

        // Optional: skip invalid experiment if path is "clear" and dynamic == initial (paper logic)
        // For demo: if it's a clear path at t=0, we skip.
        // if (isClearPath(g, initPath)) continue;   // allow clear paths also


        // Step 3: tour_static = follow initial path, but traffic changes over time steps
        double tour_static = 0.0;
        {
            vector<int> path = initPath;
            // Each hop is one "time step"
            for (int i=0; i+1<(int)path.size(); ++i) {
                simulateTraffic(g, i+1);
                // add R(t) for that edge at that moment
                int u = path[i], v = path[i+1];
                // find the edge cost
                bool found=false;
                for (int ei=0; ei<(int)g.adj[u].size(); ++ei) {
                    if (g.adj[u][ei].to == v) {
                        tour_static += edgeCost(g, u, ei, /*useRoadFactor=*/true);
                        found=true;
                        break;
                    }
                }
                if (!found) { tour_static = 1e100; break; }
            }
        }
        if (tour_static >= 1e99) continue;

        // Step 4: tour_aco = dynamic ACO with distance-only (PSO optimized params)
        // Replan each step using current traffic but weights = distance (d)
        double tour_aco = 0.0;
        {
            int cur = start;
            int t = 0;
            while (cur != goal && t < 20) {
                simulateTraffic(g, t+1);
                auto res = runACO_PointToPoint(g, cur, goal, bestParams, /*useRoadFactor=*/false, ants, acoIters);
                if (res.cost >= 1e99 || res.path.size() < 2) { tour_aco = 1e100; break; }
                // Move one hop along planned path
                int next = res.path[1];
                // add actual R(t) experienced on that hop (paper uses sum of road condition factors)
                // So we accumulate road factor even though planning used distance.
                vector<int> hop = {cur, next};
                tour_aco += pathCostOnCurrentTraffic(g, hop, /*useRoadFactor=*/true);
                cur = next;
                t++;
            }
        }
        if (tour_aco >= 1e99) continue;

        // Step 5: tour_opt = improved dynamic ACO using road factor + replan threshold
        double tour_opt = 0.0;
        {
            int cur = start;
            int t = 0;
            vector<int> currentPlan;

            while (cur != goal && t < 20) {
                simulateTraffic(g, t+1);

                // If no plan, or needs replanning, compute new plan using road factor
                if (currentPlan.empty() || currentPlan[0] != cur) {
                    auto res = runACO_PointToPoint(g, cur, goal, bestParams, /*useRoadFactor=*/true, ants, acoIters);
                    if (res.cost >= 1e99 || res.path.size() < 2) { tour_opt = 1e100; break; }
                    currentPlan = res.path;
                } else {
                    // Check if conditions changed too much on remaining path
                    double worstDeltaR=0.0;
                    if (shouldReplan(g, currentPlan, Rmax, dRmax, worstDeltaR)) {
                        auto res = runACO_PointToPoint(g, cur, goal, bestParams, /*useRoadFactor=*/true, ants, acoIters);
                        if (res.cost >= 1e99 || res.path.size() < 2) { tour_opt = 1e100; break; }
                        currentPlan = res.path;
                    }
                }

                // Move one hop along plan, add actual road factor cost
                int next = currentPlan[1];
                vector<int> hop = {cur, next};
                tour_opt += pathCostOnCurrentTraffic(g, hop, /*useRoadFactor=*/true);

                // advance
                cur = next;
                // drop first node from plan
                currentPlan.erase(currentPlan.begin());
                t++;
            }
        }
        if (tour_opt >= 1e99) continue;

        // valid result
        validCount++;
        sum_init   += tour_init;
        sum_static += tour_static;
        sum_aco    += tour_aco;
        sum_opt    += tour_opt;
    }

    if (validCount == 0) {
        cout << "No valid trials (all paths were clear or failed). Increase trials or traffic severity.\n";
        return 0;
    }

    double tour_init_aver   = sum_init   / validCount;
    double tour_static_aver = sum_static / validCount;
    double tour_aco_aver    = sum_aco    / validCount;
    double tour_opt_aver    = sum_opt    / validCount;

    // Congestion rates (paper-style)
    double CR_static   = fabs(tour_static_aver - tour_init_aver) / tour_init_aver;
    double CR_aco      = fabs(tour_aco_aver    - tour_init_aver) / tour_init_aver;
    double CR_opt      = fabs(tour_opt_aver    - tour_init_aver) / tour_init_aver;
    double CR_promote  = fabs(tour_aco_aver    - tour_opt_aver)  / tour_aco_aver;

    cout << fixed << setprecision(3);
    cout << "Valid trials: " << validCount << "\n\n";
    cout << "tour_init_aver   = " << tour_init_aver << "\n";
    cout << "tour_static_aver = " << tour_static_aver << "\n";
    cout << "tour_aco_aver    = " << tour_aco_aver << "\n";
    cout << "tour_opt_aver    = " << tour_opt_aver << "\n\n";

    cout << "CR_static   = " << (CR_static*100.0)  << " %\n";
    cout << "CR_aco      = " << (CR_aco*100.0)     << " %\n";
    cout << "CR_opt      = " << (CR_opt*100.0)     << " %\n";
    cout << "CR_promote  = " << (CR_promote*100.0) << " %\n";

    return 0;
}
