#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

using namespace std;

struct Node {
    int id;
    double g;
    double h;
    double f;
    Node(int id, double g, double h) : id(id), g(g), h(h) {
        f = g + h;
    }
};

struct CompareNodes {
    bool operator()(const Node& lhs, const Node& rhs) const {
        return lhs.f > rhs.f;
    }
};

vector<vector<double>> readMatrix(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Không thể mở file " << filename << endl;
        exit(1);
    }

    int size;
    file >> size;

    vector<vector<double>> matrix(size, vector<double>(size));

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            file >> matrix[i][j];
        }
    }

    return matrix;
}

vector<int> aStar(const vector<vector<double>>& heuristicMatrix, const vector<vector<double>>& costMatrix, int start, int goal) {
    int size = heuristicMatrix.size();
    vector<int> path;
    vector<double> gValues(size, numeric_limits<double>::infinity());
    vector<double> fValues(size, numeric_limits<double>::infinity());
    vector<int> parent(size, -1);
    priority_queue<Node, vector<Node>, CompareNodes> openSet;

    gValues[start] = 0;
    fValues[start] = heuristicMatrix[start][goal];
    openSet.push(Node(start, 0, fValues[start]));

    while (!openSet.empty()) {
        int current = openSet.top().id;
        openSet.pop();

        if (current == goal) {
            int node = goal;
            while (node != start) {
                path.push_back(node);
                node = parent[node];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor = 0; neighbor < size; neighbor++) {
            if (costMatrix[current][neighbor] > 0) {
                double tentativeG = gValues[current] + costMatrix[current][neighbor];
                if (tentativeG < gValues[neighbor]) {
                    parent[neighbor] = current;
                    gValues[neighbor] = tentativeG;
                    fValues[neighbor] = gValues[neighbor] + heuristicMatrix[neighbor][goal];
                    openSet.push(Node(neighbor, gValues[neighbor], fValues[neighbor]));
                }
            }
        }
    }

    return path; // Không tìm thấy đường đi
}

int main() {
    vector<vector<double>> heuristicMatrix = readMatrix("input1.txt");
    vector<vector<double>> costMatrix = readMatrix("input2.txt");
    int start = 0; // Đỉnh bắt đầu
    int goal = 10; // Đỉnh đích

    vector<int> path = aStar(heuristicMatrix, costMatrix, start, goal);

    if (path.empty()) {
        cout << "Khong tim thay duong di." << endl;
    }
    else {
        cout << "Duong di tu dinh " << start << " den dinh " << goal << " la: ";
        for (int node : path) {
            cout << node << " ";
        }
        cout << endl;
    }

    return 0;
}
