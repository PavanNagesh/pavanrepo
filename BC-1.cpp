#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>
#include <string>
#include <functional>
#include <map>
#include <set>
#include <queue>

using namespace std;

struct Edge {
    string src, dest;
    int weight;
};

struct Situation {
    string description;
    int intensity;
};

class Graph {
    vector<string> vertices;
    vector<Edge> edges;
    set<pair<string, string>> mstEdges;

public:
    void addVertex(const string& vertex) {
        vertices.push_back(vertex);
    }

    void addEdge(const string& src, const string& dest, int weight) {
        edges.push_back({src, dest, weight});
    }

    int getVertexIndex(const string& vertex) const {
        for (int i = 0; i < vertices.size(); ++i) {
            if (vertices[i] == vertex) {
                return i;
            }
        }
        return -1;
    }

    pair<vector<string>, int> bellmanFordWithPath(const string& src, const string& dest, bool blockBF = false) {
        int V = vertices.size();
        int srcIndex = getVertexIndex(src);
        int destIndex = getVertexIndex(dest);

        if (srcIndex == -1 || destIndex == -1) {
            return {{}, -1};
        }

        vector<int> dist(V, INT_MAX);
        vector<int> prev(V, -1);
        dist[srcIndex] = 0;

        for (int i = 1; i <= V - 1; ++i) {
            for (const auto& edge : edges) {
                if (blockBF && edge.src == "B" && edge.dest == "F") continue;

                int u = getVertexIndex(edge.src);
                int v = getVertexIndex(edge.dest);
                int weight = edge.weight;

                if (u != -1 && v != -1 && dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                }
            }
        }

        vector<string> path;
        int current = destIndex;
        while (current != -1) {
            path.push_back(vertices[current]);
            current = prev[current];
        }
        reverse(path.begin(), path.end());

        if (path.empty() || path[0] != src) return {{}, -1};

        return {path, dist[destIndex]};
    }

    void kruskal() {
        int V = vertices.size();
        if (V == 0) {
            cout << "Graph has no vertices.\n";
            return;
        }

        sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
            return a.weight < b.weight;
        });

        vector<int> p(V);
        for (int i = 0; i < V; ++i) p[i] = i;

        function<int(int)> find = [&](int x) {
            if (x == p[x]) return x;
            return p[x] = find(p[x]);
        };

        auto unite = [&](int x, int y) {
            int rootX = find(x);
            int rootY = find(y);
            if (rootX != rootY) p[rootX] = rootY;
        };

        mstEdges.clear();
        for (const auto& edge : edges) {
            int u = getVertexIndex(edge.src);
            int v = getVertexIndex(edge.dest);
            if (find(u) != find(v)) {
                unite(u, v);
                mstEdges.insert({edge.src, edge.dest});
                mstEdges.insert({edge.dest, edge.src});
            }
        }
    }

    pair<vector<string>, int> findPathInMST(const string& start, const string& end) {
        int startIndex = getVertexIndex(start);
        int endIndex = getVertexIndex(end);

        if (startIndex == -1 || endIndex == -1) {
            return {{}, -1};
        }

        map<string, string> parent;
        map<string, int> dist;
        queue<string> q;
        q.push(start);
        dist[start] = 0;

        while (!q.empty()) {
            string u = q.front();
            q.pop();
            if (u == end) break;
            for (const auto& edge : edges) {
                if (mstEdges.count({edge.src, edge.dest})) {
                    if (edge.src == u && !dist.count(edge.dest)) {
                        dist[edge.dest] = dist[u] + edge.weight;
                        parent[edge.dest] = u;
                        q.push(edge.dest);
                    } else if (edge.dest == u && !dist.count(edge.src)) {
                        dist[edge.src] = dist[u] + edge.weight;
                        parent[edge.src] = u;
                        q.push(edge.src);
                    }
                }
            }
        }
        if (!dist.count(end)) return {{}, -1};
        vector<string> path;
        string current = end;
        int distance = dist[end];
        while (current != start) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return {path, distance};
    }
};

void quickSort(vector<Situation>& situations, int low, int high) {
    auto partition = [&](int low, int high) {
        int pivot = situations[high].intensity;
        int i = low - 1;
        for (int j = low; j < high; ++j) {
            if (situations[j].intensity > pivot) {
                swap(situations[++i], situations[j]);
            }
        }
        swap(situations[i + 1], situations[high]);
        return i + 1;
    };

    if (low < high) {
        int pi = partition(low, high);
        quickSort(situations, low, pi - 1);
        quickSort(situations, pi + 1, high);
    }
}

int main() {
    Graph g;
    g.addVertex("Hospital");
    g.addVertex("A");
    g.addVertex("B");
    g.addVertex("C");
    g.addVertex("D");
    g.addVertex("E");
    g.addVertex("F");
    g.addVertex("Residential Area");

    g.addEdge("Hospital", "A", 3);
    g.addEdge("A", "B", 2);
    g.addEdge("A", "D", 4);
    g.addEdge("B", "C", 3);
    g.addEdge("B", "F", 4);
    g.addEdge("D", "E", 2);
    g.addEdge("E", "F", 1);
    g.addEdge("F", "Residential Area", 1);
    g.addEdge("C", "Residential Area", 4);

    cout << "Running Kruskal's Algorithm (Quickest Path/MST):\n";
    g.kruskal();

    pair<vector<string>, int> pathInfoMST = g.findPathInMST("Hospital", "Residential Area");

    if (pathInfoMST.second != -1) {
        cout << "\nShortest path from Hospital to Residential Area (using MST):\n";
        for (const string& vertex : pathInfoMST.first) {
            cout << vertex << " -> ";
        }
        cout << "\b\b\b   \n";
        cout << "Total time: " << pathInfoMST.second << " minutes\n";
    } else {
        cout << "\nNo path found between Hospital and Residential Area in the MST.\n";
    }

    cout << "\nRunning Bellman-Ford Algorithm (Normal Conditions):\n";
    pair<vector<string>, int> pathInfoBFNormal = g.bellmanFordWithPath("Hospital", "Residential Area");
    if (pathInfoBFNormal.second != -1) {
        cout << "Path: ";
        for (const string& vertex : pathInfoBFNormal.first) {
            cout << vertex << " -> ";
        }
        cout << "\b\b\b   \n";
        cout << "Total time: " << pathInfoBFNormal.second << " minutes\n";
    } else {
        cout << "No Path Found\n";
    }

    cout << "\nRunning Bellman-Ford Algorithm (B-F Path Blocked - Emergency):\n";
    pair<vector<string>, int> pathInfoBFBlocked = g.bellmanFordWithPath("Hospital", "Residential Area", true);
    if (pathInfoBFBlocked.second != -1) {
        cout << "Path: ";
        for (const string& vertex : pathInfoBFBlocked.first) {
            cout << vertex << " -> ";
        }
        cout << "\b\b\b   \n";
        cout << "Total time: " << pathInfoBFBlocked.second << " minutes\n";
    } else {
        cout << "No Path Found\n";
    }

    vector<Situation> situations = {
        {"Travel from Hospital to Residential Area", 10},
        {"Travel from Hospital to Industrial Area", 5},
        {"Building Collapse at B", 15},
        {"Flooding near F", 12}
    };

    quickSort(situations, 0, situations.size() - 1);

    cout << "\nSituations sorted by intensity (Descending):\n";
    for (const auto& situation : situations) {
        cout << situation.description << " - Intensity: " << situation.intensity << "\n";
    }

    return 0;
}
