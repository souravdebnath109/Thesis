//======
// Here is method.cpp.

// It does these things:

// reads config.txt
// reads input.csv
// builds the road network from link rows
// reads vehicles from vehicle rows
//===========================
// uses a decomposition-style iterative method
// route vehicles
// measure edge load
// increase cost on congested edges
// optionally apply sudden blockage penalty
// reroute again for several iterations
//===================
// saves final result in output.csv
// output format:
// vehicle,path
// example: 1,1-2-3-4
//======

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <algorithm>
#include <random>
#include <functional>

using namespace std;

struct Config {
    int NUM_NODES = 100;
    int NUM_LINKS = 1000;
    int NUM_VEHICLES = 10000;
    int NUM_ITERATIONS = 20;
    int RANDOM_SEED = 42;

    double BASE_EDGE_COST = 1.0;
    double CONGESTION_ALPHA = 0.15;
    double BLOCKAGE_PROBABILITY = 0.01;
    double BLOCKAGE_PENALTY = 1000000.0;
};

struct Edge {
    int id;
    int from;
    int to;
    double baseCost;
    double currentCost;
    int load;
    bool blocked;
};

struct Vehicle {
    int id;
    int source;
    int destination;
};

struct PathResult {
    bool found;
    vector<int> nodes;
    vector<int> edgeIds;
    double cost;

    PathResult() {
        found = false;
        cost = 0.0;
    }
};

static string trim(const string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

bool loadConfig(const string& filename, Config& config) {
    ifstream fin(filename.c_str());
    if (!fin.is_open()) {
        cerr << "Error: could not open config file: " << filename << endl;
        return false;
    }

    string line;
    while (getline(fin, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        size_t pos = line.find('=');
        if (pos == string::npos) continue;

        string key = trim(line.substr(0, pos));
        string value = trim(line.substr(pos + 1));

        try {
            if (key == "NUM_NODES") config.NUM_NODES = stoi(value);
            else if (key == "NUM_LINKS") config.NUM_LINKS = stoi(value);
            else if (key == "NUM_VEHICLES") config.NUM_VEHICLES = stoi(value);
            else if (key == "NUM_ITERATIONS") config.NUM_ITERATIONS = stoi(value);
            else if (key == "RANDOM_SEED") config.RANDOM_SEED = stoi(value);
            else if (key == "BASE_EDGE_COST") config.BASE_EDGE_COST = stod(value);
            else if (key == "CONGESTION_ALPHA") config.CONGESTION_ALPHA = stod(value);
            else if (key == "BLOCKAGE_PROBABILITY") config.BLOCKAGE_PROBABILITY = stod(value);
            else if (key == "BLOCKAGE_PENALTY") config.BLOCKAGE_PENALTY = stod(value);
        } catch (...) {
            cerr << "Warning: invalid value for key " << key << endl;
        }
    }

    fin.close();
    return true;
}

bool loadInputCSV(
    const string& filename,
    const Config& config,
    vector<Edge>& edges,
    vector<Vehicle>& vehicles,
    vector<vector<int> >& adj
) {
    ifstream fin(filename.c_str());
    if (!fin.is_open()) {
        cerr << "Error: could not open input file: " << filename << endl;
        return false;
    }

    adj.assign(config.NUM_NODES + 1, vector<int>());
    edges.clear();
    vehicles.clear();

    string line;
    bool firstLine = true;
    int edgeCounter = 0;

    while (getline(fin, line)) {
        line = trim(line);
        if (line.empty()) continue;

        if (firstLine) {
            firstLine = false;
            if (line == "type,id,from,to") continue;
        }

        stringstream ss(line);
        string typeStr, idStr, fromStr, toStr;

        if (!getline(ss, typeStr, ',')) continue;
        if (!getline(ss, idStr, ',')) continue;
        if (!getline(ss, fromStr, ',')) continue;
        if (!getline(ss, toStr, ',')) continue;

        string type = trim(typeStr);
        int id = stoi(trim(idStr));
        int from = stoi(trim(fromStr));
        int to = stoi(trim(toStr));

        if (from < 1 || from > config.NUM_NODES || to < 1 || to > config.NUM_NODES) {
            continue;
        }

        if (type == "link") {
            Edge e;
            e.id = edgeCounter;
            e.from = from;
            e.to = to;
            e.baseCost = config.BASE_EDGE_COST;
            e.currentCost = config.BASE_EDGE_COST;
            e.load = 0;
            e.blocked = false;

            edges.push_back(e);
            adj[from].push_back(edgeCounter);
            edgeCounter++;
        } else if (type == "vehicle") {
            Vehicle v;
            v.id = id;
            v.source = from;
            v.destination = to;
            vehicles.push_back(v);
        }
    }

    fin.close();
    return true;
}

PathResult shortestPath(
    int source,
    int target,
    const vector<Edge>& edges,
    const vector<vector<int> >& adj,
    int numNodes
) {
    const double INF = numeric_limits<double>::max();

    vector<double> dist(numNodes + 1, INF);
    vector<int> parentNode(numNodes + 1, -1);
    vector<int> parentEdge(numNodes + 1, -1);

    priority_queue< pair<double, int>, vector< pair<double, int> >, greater< pair<double, int> > > pq;

    dist[source] = 0.0;
    pq.push(make_pair(0.0, source));

    while (!pq.empty()) {
        pair<double, int> topItem = pq.top();
        pq.pop();

        double d = topItem.first;
        int u = topItem.second;

        if (d > dist[u]) continue;
        if (u == target) break;

        for (size_t i = 0; i < adj[u].size(); ++i) {
            int edgeId = adj[u][i];
            const Edge& e = edges[edgeId];

            double nd = dist[u] + e.currentCost;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                parentNode[e.to] = u;
                parentEdge[e.to] = edgeId;
                pq.push(make_pair(nd, e.to));
            }
        }
    }

    PathResult result;
    if (dist[target] >= INF / 2.0) {
        result.found = false;
        return result;
    }

    result.found = true;
    result.cost = dist[target];

    vector<int> revNodes;
    vector<int> revEdges;

    int cur = target;
    revNodes.push_back(cur);

    while (cur != source) {
        int eId = parentEdge[cur];
        int p = parentNode[cur];

        if (eId == -1 || p == -1) {
            result.found = false;
            result.nodes.clear();
            result.edgeIds.clear();
            return result;
        }

        revEdges.push_back(eId);
        cur = p;
        revNodes.push_back(cur);
    }

    reverse(revNodes.begin(), revNodes.end());
    reverse(revEdges.begin(), revEdges.end());

    result.nodes = revNodes;
    result.edgeIds = revEdges;
    return result;
}

string pathToString(const vector<int>& nodes) {
    if (nodes.empty()) return "NO_PATH";

    string s;
    for (size_t i = 0; i < nodes.size(); ++i) {
        s += to_string(nodes[i]);
        if (i + 1 < nodes.size()) s += "-";
    }
    return s;
}

int main() {
    Config config;

    if (!loadConfig("config.txt", config)) {
        return 1;
    }

    vector<Edge> edges;
    vector<Vehicle> vehicles;
    vector<vector<int> > adj;

    if (!loadInputCSV("input.csv", config, edges, vehicles, adj)) {
        return 1;
    }

    if (edges.empty()) {
        cerr << "Error: no links found in input.csv" << endl;
        return 1;
    }

    if (vehicles.empty()) {
        cerr << "Error: no vehicles found in input.csv" << endl;
        return 1;
    }

    mt19937 rng(config.RANDOM_SEED);
    uniform_real_distribution<double> probDist(0.0, 1.0);

    vector<PathResult> bestPathForVehicle(vehicles.size());
    vector<string> finalPathStrings(vehicles.size(), "NO_PATH");
    double bestObjective = numeric_limits<double>::max();

    for (int iter = 0; iter < config.NUM_ITERATIONS; ++iter) {
        size_t i;

        for (i = 0; i < edges.size(); ++i) {
            edges[i].load = 0;
            edges[i].blocked = false;
        }

        for (i = 0; i < edges.size(); ++i) {
            if (probDist(rng) < config.BLOCKAGE_PROBABILITY) {
                edges[i].blocked = true;
            }
        }

        vector<PathResult> currentPaths(vehicles.size());
        double totalCost = 0.0;
        int noPathCount = 0;

        for (i = 0; i < vehicles.size(); ++i) {
            currentPaths[i] = shortestPath(
                vehicles[i].source,
                vehicles[i].destination,
                edges,
                adj,
                config.NUM_NODES
            );

            if (!currentPaths[i].found) {
                noPathCount++;
                totalCost += config.BLOCKAGE_PENALTY;
                continue;
            }

            totalCost += currentPaths[i].cost;

            for (size_t j = 0; j < currentPaths[i].edgeIds.size(); ++j) {
                int edgeId = currentPaths[i].edgeIds[j];
                edges[edgeId].load++;
            }
        }

        for (i = 0; i < edges.size(); ++i) {
            double congestionFactor = 1.0 + config.CONGESTION_ALPHA * (double)edges[i].load;
            edges[i].currentCost = edges[i].baseCost * congestionFactor;

            if (edges[i].blocked) {
                edges[i].currentCost += config.BLOCKAGE_PENALTY;
            }
        }

        vector<PathResult> reroutedPaths(vehicles.size());
        double reroutedTotalCost = 0.0;
        int reroutedNoPathCount = 0;

        for (i = 0; i < vehicles.size(); ++i) {
            reroutedPaths[i] = shortestPath(
                vehicles[i].source,
                vehicles[i].destination,
                edges,
                adj,
                config.NUM_NODES
            );

            if (!reroutedPaths[i].found) {
                reroutedNoPathCount++;
                reroutedTotalCost += config.BLOCKAGE_PENALTY;
                continue;
            }

            reroutedTotalCost += reroutedPaths[i].cost;
        }

        bool useRerouted = false;
        if (reroutedNoPathCount < noPathCount) {
            useRerouted = true;
        } else if (reroutedNoPathCount == noPathCount && reroutedTotalCost < totalCost) {
            useRerouted = true;
        }

        double candidateObjective = useRerouted ? reroutedTotalCost : totalCost;
        const vector<PathResult>& candidatePaths = useRerouted ? reroutedPaths : currentPaths;
        int candidateNoPath = useRerouted ? reroutedNoPathCount : noPathCount;

        candidateObjective += 1000.0 * candidateNoPath;

        if (candidateObjective < bestObjective) {
            bestObjective = candidateObjective;

            for (i = 0; i < vehicles.size(); ++i) {
                bestPathForVehicle[i] = candidatePaths[i];

                if (bestPathForVehicle[i].found) {
                    finalPathStrings[i] = pathToString(bestPathForVehicle[i].nodes);
                } else {
                    finalPathStrings[i] = "NO_PATH";
                }
            }
        }
    }

    ofstream fout("output.csv");
    if (!fout.is_open()) {
        cerr << "Error: could not create output.csv" << endl;
        return 1;
    }

    fout << "vehicle,path\n";
    for (size_t i = 0; i < vehicles.size(); ++i) {
        fout << vehicles[i].id << "," << finalPathStrings[i] << "\n";
    }

    fout.close();

    cout << "Solved using decomposition-style iterative routing." << endl;
    cout << "Input file  : input.csv" << endl;
    cout << "Output file : output.csv" << endl;
    cout << "Vehicles    : " << vehicles.size() << endl;
    cout << "Links       : " << edges.size() << endl;
    cout << "Iterations  : " << config.NUM_ITERATIONS << endl;

    return 0;
}