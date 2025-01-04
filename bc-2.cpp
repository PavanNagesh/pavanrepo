#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <map>
using namespace std;

// Structure to represent an edge
struct Edge {
    string src, dest;
    int weight;
};

// Structure to represent a subset for union-find
struct Subset {
    string parent;
    int rank;
};

// Function to find the parent of a node in the subset
string findParent(map<string, Subset>& subsets, const string& node) {
    if (subsets[node].parent != node) {
        subsets[node].parent = findParent(subsets, subsets[node].parent);
    }
    return subsets[node].parent;
}

// Union of two subsets based on rank
void unionSubsets(map<string, Subset>& subsets, const string& x, const string& y) {
    string xRoot = findParent(subsets, x);
    string yRoot = findParent(subsets, y);

    if (subsets[xRoot].rank < subsets[yRoot].rank) {
        subsets[xRoot].parent = yRoot;
    } else if (subsets[xRoot].rank > subsets[yRoot].rank) {
        subsets[yRoot].parent = xRoot;
    } else {
        subsets[yRoot].parent = xRoot;
        subsets[xRoot].rank++;
    }
}

// Function to implement Kruskal's algorithm
void kruskalMST(const vector<Edge>& edges, int numNodes) {
    vector<Edge> result; // Stores the resultant MST
    map<string, Subset> subsets; // Subsets for union-find

    // Initialize subsets
    for (const Edge& edge : edges) {
        subsets[edge.src] = {edge.src, 0};
        subsets[edge.dest] = {edge.dest, 0};
    }

    // Sort edges in ascending order of weights
    vector<Edge> sortedEdges = edges;
    sort(sortedEdges.begin(), sortedEdges.end(), [](const Edge& a, const Edge& b) {
        return a.weight < b.weight;
    });

    for (const Edge& edge : sortedEdges) {
        string x = findParent(subsets, edge.src);
        string y = findParent(subsets, edge.dest);

        if (x != y) {
            result.push_back(edge);
            unionSubsets(subsets, x, y);
        }
    }

    // Display the MST
    cout << "\nMinimum Spanning Tree (MST) using Kruskal's Algorithm:" << endl;
    cout << "Edge \t Weight" << endl;
    int totalWeight = 0;
    for (const Edge& edge : result) {
        cout << edge.src << " - " << edge.dest << " \t " << edge.weight << " km" << endl;
        totalWeight += edge.weight;
    }
    cout << "\nTotal weight of MST: " << totalWeight << " km\n" << endl;
}

// Red-Black Tree Node structure
struct RBNode {
    string name;
    string color;
    RBNode* left;
    RBNode* right;
    RBNode* parent;
    bool isPowerOn;

    RBNode(string name, bool isPowerOn)
        : name(name), color("RED"), left(nullptr), right(nullptr), parent(nullptr), isPowerOn(isPowerOn) {}
};

class RedBlackTree {
private:
    RBNode* root;
    RBNode* TNULL;

    void initializeNullNode(RBNode* node, RBNode* parent) {
        node->name = "";
        node->color = "BLACK";
        node->left = nullptr;
        node->right = nullptr;
        node->parent = parent;
        node->isPowerOn = true;
    }

    void printHelper(RBNode* node, string indent, bool last) {
        if (node != TNULL) {
            cout << indent;
            if (last) {
                cout << "R----";
                indent += "   ";
            } else {
                cout << "L----";
                indent += "|  ";
            }

            string status = node->isPowerOn ? "ON" : "OFF";
            cout << node->name << "(" << node->color << ", " << status << ")" << endl;
            printHelper(node->left, indent, false);
            printHelper(node->right, indent, true);
        }
    }

public:
    RedBlackTree() {
        TNULL = new RBNode("", true);
        TNULL->color = "BLACK";
        root = TNULL;
    }

    void insert(string key, bool isPowerOn) {
        RBNode* node = new RBNode(key, isPowerOn);
        node->parent = nullptr;
        node->name = key;
        node->isPowerOn = isPowerOn;
        node->left = TNULL;
        node->right = TNULL;
        node->color = "RED";

        RBNode* y = nullptr;
        RBNode* x = this->root;

        while (x != TNULL) {
            y = x;
            if (node->name < x->name) {
                x = x->left;
            } else {
                x = x->right;
            }
        }

        node->parent = y;
        if (y == nullptr) {
            root = node;
        } else if (node->name < y->name) {
            y->left = node;
        } else {
            y->right = node;
        }

        if (node->parent == nullptr) {
            node->color = "BLACK";
            return;
        }

        if (node->parent->parent == nullptr) {
            return;
        }

        // Balance the tree after insertion
        balanceInsert(node);
    }

    void balanceInsert(RBNode* k) {
        RBNode* u;
        while (k->parent->color == "RED") {
            if (k->parent == k->parent->parent->right) {
                u = k->parent->parent->left;
                if (u->color == "RED") {
                    u->color = "BLACK";
                    k->parent->color = "BLACK";
                    k->parent->parent->color = "RED";
                    k = k->parent->parent;
                } else {
                    if (k == k->parent->left) {
                        k = k->parent;
                        rightRotate(k);
                    }
                    k->parent->color = "BLACK";
                    k->parent->parent->color = "RED";
                    leftRotate(k->parent->parent);
                }
            } else {
                u = k->parent->parent->right;

                if (u->color == "RED") {
                    u->color = "BLACK";
                    k->parent->color = "BLACK";
                    k->parent->parent->color = "RED";
                    k = k->parent->parent;
                } else {
                    if (k == k->parent->right) {
                        k = k->parent;
                        leftRotate(k);
                    }
                    k->parent->color = "BLACK";
                    k->parent->parent->color = "RED";
                    rightRotate(k->parent->parent);
                }
            }
            if (k == root) {
                break;
            }
        }
        root->color = "BLACK";
    }

    void leftRotate(RBNode* x) {
        RBNode* y = x->right;
        x->right = y->left;
        if (y->left != TNULL) {
            y->left->parent = x;
        }
        y->parent = x->parent;
        if (x->parent == nullptr) {
            this->root = y;
        } else if (x == x->parent->left) {
            x->parent->left = y;
        } else {
            x->parent->right = y;
        }
        y->left = x;
        x->parent = y;
    }

    void rightRotate(RBNode* x) {
        RBNode* y = x->left;
        x->left = y->right;
        if (y->right != TNULL) {
            y->right->parent = x;
        }
        y->parent = x->parent;
        if (x->parent == nullptr) {
            this->root = y;
        } else if (x == x->parent->right) {
            x->parent->right = y;
        } else {
            x->parent->left = y;
        }
        y->right = x;
        x->parent = y;
    }

    void display() {
        if (root) {
            printHelper(this->root, "", true);
        }
    }

    void alert(string nodeName) {
        RBNode* node = search(this->root, nodeName);
        if (node != nullptr && !node->isPowerOn) {
            cout << "ALERT: Power supply to " << nodeName << " is OFF. Officials have been notified!\n";
        }
    }

    RBNode* search(RBNode* node, string key) {
        if (node == TNULL || key == node->name) {
            return node;
        }

        if (key < node->name) {
            return search(node->left, key);
        }
        return search(node->right, key);
    }
};

int main() {
    // Define the graph edges with weights
    vector<Edge> edges = {
        {"Electricity Supply", "A", 5},
        {"Electricity Supply", "B", 7},
        {"Electricity Supply", "C", 9},
        {"Electricity Supply", "D", 10},
        {"Electricity Supply", "E", 14},
        {"A", "B", 6},
        {"B", "C", 4},
        {"C", "D", 5},
        {"D", "E", 6}
    };

    int numNodes = 6; // Number of nodes including the Electricity Supply

    // Run Kruskal's algorithm to find MST
    kruskalMST(edges, numNodes);

    // Create and initialize the Red-Black Tree
    RedBlackTree rbTree;
    rbTree.insert("Electricity Supply", true);
    rbTree.insert("A", true);
    rbTree.insert("B", true);
    rbTree.insert("C", true);
    rbTree.insert("D", false); // Simulating power cut at D
    rbTree.insert("E", true);

    // Display the Red-Black Tree
    cout << "\nRed-Black Tree Status:\n";
    rbTree.display();

    // Simulate alert for node D
    rbTree.alert("D");

    return 0;
}
