#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <string>
#include <random>
#include <algorithm>

using namespace std;

struct Config {
    int NUM_NODES = 100;
    int NUM_LINKS = 1000;
    int NUM_VEHICLES = 10000;
    int NUM_ITERATIONS = 50;
    int RANDOM_SEED = 42;
};

static string trim(const string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

bool loadConfig(const string& filename, Config& config) {
    ifstream fin(filename);
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
        } catch (...) {
            cerr << "Warning: invalid value for key " << key << " in config file." << endl;
        }
    }

    fin.close();
    return true;
}

int main() {
    Config config;
    if (!loadConfig("config.txt", config)) {
        return 1;
    }

    if (config.NUM_NODES < 2) {
        cerr << "Error: NUM_NODES must be at least 2." << endl;
        return 1;
    }

    // Directed simple graph max links = N*(N-1)
    long long maxPossibleLinks = 1LL * config.NUM_NODES * (config.NUM_NODES - 1);
    if (config.NUM_LINKS > maxPossibleLinks) {
        cerr << "Error: NUM_LINKS is too large for a directed graph with "
             << config.NUM_NODES << " nodes." << endl;
        cerr << "Maximum possible directed links without self-loops = "
             << maxPossibleLinks << endl;
        return 1;
    }

    mt19937 rng(config.RANDOM_SEED);
    uniform_int_distribution<int> nodeDist(1, config.NUM_NODES);

    ofstream fout("input.csv");
    if (!fout.is_open()) {
        cerr << "Error: could not create input.csv" << endl;
        return 1;
    }

    // CSV header
    fout << "type,id,from,to\n";

    // -----------------------------
    // Generate unique directed links
    // -----------------------------
    unordered_set<long long> usedLinks;
    int linkCount = 0;

    while (linkCount < config.NUM_LINKS) {
        int u = nodeDist(rng);
        int v = nodeDist(rng);

        if (u == v) continue;

        long long key = 1LL * u * 1000000LL + v;
        if (usedLinks.find(key) != usedLinks.end()) continue;

        usedLinks.insert(key);
        ++linkCount;
        fout << "link," << linkCount << "," << u << "," << v << "\n";
    }

    // -----------------------------
    // Generate vehicles
    // -----------------------------
    int vehicleCount = 0;
    while (vehicleCount < config.NUM_VEHICLES) {
        int source = nodeDist(rng);
        int destination = nodeDist(rng);

        if (source == destination) continue;

        ++vehicleCount;
        fout << "vehicle," << vehicleCount << "," << source << "," << destination << "\n";
    }

    fout.close();

    cout << "input generated successfully: input.csv" << endl;
    cout << "Nodes: " << config.NUM_NODES << endl;
    cout << "Links: " << config.NUM_LINKS << endl;
    cout << "Vehicles: " << config.NUM_VEHICLES << endl;

    return 0;
}