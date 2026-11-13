// Starlink Beam Planning Tech Test - standalone C++17 solver.
// Usage: ./starlink_beam_planner input.txt > solution.txt
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator-(const Vec3& b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3 operator/(double k) const { return {x/k,y/k,z/k}; }
};
static inline double dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline double length(const Vec3& a) { return sqrt(dot(a,a)); }

struct Object { string id; Vec3 pos; };
struct Candidate { int sat; Vec3 beam; }; // unit direction: satellite -> user
struct Assignment { int user, sat, color; };

int main(int argc, char** argv) {
    // ios::sync_with_stdio(false);
    // cin.tie(nullptr);
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <input-file>\n";
        return 1;
    }
    ifstream input(argv[1]);
    if (!input) { cerr << "Cannot open input: " << argv[1] << '\n'; return 1; }

    vector<Object> users, sats, interferers;
    string line, kind, id;
    while (getline(input, line)) {
        auto hash = line.find('#');
        if (hash != string::npos) line.erase(hash);
        istringstream ss(line);
        Object obj;
        if (!(ss >> kind >> obj.id >> obj.pos.x >> obj.pos.y >> obj.pos.z)) continue;
        if (kind == "user") users.push_back(obj);
        else if (kind == "sat") sats.push_back(obj);
        else if (kind == "interferer") interferers.push_back(obj);
    }

    const double PI = acos(-1.0);
    const double C45 = cos(PI/4.0);
    const double C20 = cos(PI/9.0);
    const double C10 = cos(PI/18.0);
    // Strict margins prevent floating-point rounding errors at an angle boundary.
    const double EPS = 1e-12;
    const int n = static_cast<int>(users.size());
    const int m = static_cast<int>(sats.size());
    if (n == 0 || m == 0) return 0;

    // Precompute directions to interfering satellites from each user.
    vector<vector<Vec3>> interfererDirs(n);
    for (int u = 0; u < n; ++u) {
        for (const Object& intr : interferers) {
            Vec3 d = intr.pos - users[u].pos;
            double norm = length(d);
            if (norm > 0) interfererDirs[u].push_back(d / norm);
        }
    }

    vector<vector<Candidate>> choices(n);
    for (int u = 0; u < n; ++u) {
        double userRadius = length(users[u].pos);
        if (userRadius <= 0) continue;
        Vec3 vertical = users[u].pos / userRadius;
        for (int s = 0; s < m; ++s) {
            Vec3 toSat = sats[s].pos - users[u].pos;
            double d = length(toSat);
            if (d <= 0) continue;
            Vec3 fromUser = toSat / d;
            // The link is no more than 45 degrees from the local vertical.
            if (dot(fromUser, vertical) < C45 + EPS) continue;
            bool interferes = false;
            for (const Vec3& other : interfererDirs[u]) {
                // The link must stay at least 20 degrees away from interferers.
                if (dot(fromUser, other) > C20 - EPS) {
                    interferes = true;
                    break;
                }
            }
            if (!interferes) choices[u].push_back({s, -fromUser});
        }
    }

    vector<int> order;
    order.reserve(n);
    for (int u = 0; u < n; ++u) if (!choices[u].empty()) order.push_back(u);
    if (order.empty()) return 0;

    // Multiple low-cost attempts can improve coverage over a single greedy pass.
    // const int attempts = (order.size() > 50000 ? 2 : order.size() > 10000 ? 4 : 10);
    // mt19937 rng(0x5a17b1eU);
    vector<Assignment> best;

    //for (int attempt = 0; attempt < attempts; ++attempt) {
        // shuffle(order.begin(), order.end(), rng);
        stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return choices[a].size() < choices[b].size();
        });

        vector<array<int, 32>> beamUser(m);
        vector<array<Vec3, 32>> beamDir(m);
        vector<array<unsigned char, 32>> beamColor(m);
        vector<int> load(m, 0);
        vector<Assignment> result;
        result.reserve(min<size_t>(order.size(), static_cast<size_t>(m)*32));

        for (int u : order) {
            int chosenSat = -1, chosenColor = -1;
            //double chosenScore = numeric_limits<double>::infinity();

            for (const Candidate& c : choices[u]) {
                int s = c.sat;
                if (load[s] == 32) continue;
                int forbidden = 0;
                for (int k = 0; k < load[s]; ++k) {
                    // Close beams may exist, but they MUST have different colors.
                    if (dot(c.beam, beamDir[s][k]) > C10 - EPS)
                        forbidden |= 1 << beamColor[s][k];
                }
                if (forbidden == 15) continue;

                int color = -1;
                int smallestUsage = 1000;
                for (int t = 0; t < 4; ++t) {
                    if (forbidden & (1 << t)) continue;
                    int usage = 0;
                    for (int k = 0; k < load[s]; ++k)
                        usage += beamColor[s][k] == t;
                    if (usage < smallestUsage) {
                        smallestUsage = usage;
                        color = t;
                    }
                }

                // Balance load between satellites. Favor colors that retain flexibility.
                // double jitter = static_cast<double>(rng() % 1000) / 1000.0;
                //double score = 4.0*load[s] + smallestUsage + jitter*3.0;
                //if (score < chosenScore) {
                    //chosenScore = score;
                    chosenSat = s;
                    chosenColor = color;
                //}
            }

            if (chosenSat < 0) continue;
            int k = load[chosenSat]++;
            beamUser[chosenSat][k] = u;
            beamColor[chosenSat][k] = static_cast<unsigned char>(chosenColor);
            // Retrieve corresponding direction for the selected satellite.
            for (const Candidate& c : choices[u]) {
                if (c.sat == chosenSat) { beamDir[chosenSat][k] = c.beam; break; }
            }
            result.push_back({u, chosenSat, chosenColor});
        }
        if (result.size() > best.size()) best = std::move(result);
    //}

    vector<int> beamCount(m, 0);
    for (const auto& a : best) {
        cout << "sat " << sats[a.sat].id
             << " beam " << ++beamCount[a.sat]
             << " user " << users[a.user].id
             << " color " << static_cast<char>('A' + a.color) << '\n';
    }
    return 0;
}
