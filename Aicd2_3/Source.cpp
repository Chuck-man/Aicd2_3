#include <iostream>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <iterator>
#include <string>
#include <queue>
#include <limits>
#include <functional>
#include <random>

using namespace std;

template<typename Vertex, typename Distance = double>

class Graph {
public:

    struct Edge {
        Vertex from, to;
        Distance distance;
    };

    Graph() {};

    ~Graph()
    {
        _data.clear();
    }

    //проверка-добавление-удаление вершин
    bool has_vertex(const Vertex& v) const {
        for (const auto& vertex : _data) {
            if (v == vertex.first) { return true; }
        }
        return false;
    }

    void add_vertex(const Vertex& v) {
        if (has_vertex(v)) return;
        _data.insert({ v, {} });
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v))
            return false;

        _data.erase(v);

        for (auto& v : _data) {
            auto& edges = v.second;
            edges.erase(std::remove_if(edges.begin(), edges.end(),
                [&v](const Edge& e) { return e.to == v; }),
                edges.end());
        }

        return true;
    }

    std::vector<Vertex> vertices() const {
        vector<Vertex> result;
        for (auto& i : _data) {
            result.push_back(i.first);
        }
        return result;
    }

    //проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d)
    {
        if (!has_vertex(from) || !has_vertex(to)) throw invalid_argument("Graph does not have a given vertex");
        if (from == to) _data[from].push_back({ from, to, d });
        else
        {
            _data.find(from)->second.push_back({ from, to, d });
        }
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (!has_vertex(from) or !has_vertex(to)) { return false; }
        auto& edges = _data.find(from)->second;
        auto it = edges.begin();
        bool res = false;

        while (it != edges.end()) {
            if (it->from == from and it->to == to) {
                it = edges.erase(it);
                res = true;
            }
            else { ++it; }
        }
        return res;
    }
    bool remove_edge(const Edge& e) {
        if (!has_vertex(e.from) or !has_vertex(e.to)) { return false; }
        auto& edges = _data.find(e.from)->second;
        auto it = edges.begin();
        bool res = false;

        while (it != edges.end()) {
            if (it->from == e.from and it->to == e.to and it->distance == e.distance) {
                it = edges.erase(it);
                res = true;
            }
            else { ++it; }
        }
        return res;
    }

    bool has_edge(const Vertex& from, const Vertex& to)  {
        if (!has_vertex(from) or !has_vertex(to)) { return false; }
        for (auto& edge : edges(from)) {
            if (edge.from == from && edge.to == to) { return true; }
        }
        return false;
    }

    bool has_edge(const Edge& e) {
        if (!has_vertex(e.from) or !has_vertex(e.to)) { return false; }
        for (auto& edge : edges(e.from)) {
            if (edge.from == e.from and edge.to == e.to and edge.distance == e.distance) { return true; }
        }
        return false;
    } //c учетом расстояния в Edge

    //получение всех ребер, выходящих из вершины
    std::vector<Edge> edges(const Vertex& vertex) {
        if (!has_vertex(vertex)) throw std::invalid_argument("The graph has no given vertex!");
        vector<Edge> result;
        auto& edges = _data.find(vertex)->second;
        auto it = edges.begin();

        while (it != edges.end()) {
            result.push_back(*it);
            ++it;
        }
        return result;
    }

    size_t order() const {

        return _data.size();
    } //порядок
    size_t degree(const Vertex& vertex) const {

        if (!has_vertex(vertex)) throw invalid_argument("Graph does not have a given vertex");
        return _data[vertex].size();
    } //степень

    //поиск кратчайшего пути
    vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        unordered_map<Vertex, Distance> distances;
        unordered_map<Vertex, Vertex> predecessors;

        for (const auto& v : _data) {
            distances[v.first] = numeric_limits<Distance>::infinity();
        }

        distances[from] = 0;

        for (size_t i = 0; i < _data.size() - 1; ++i) {
            for (const auto& vertices : _data) {
                for (const auto& edges : vertices.second) {
                    if (distances[edges.from] + edges.distance < distances[edges.to]) {
                        distances[edges.to] = distances[edges.from] + edges.distance;
                        predecessors[edges.to] = edges.from;
                    }
                }
            }
        }

        vector<Vertex> path;
        Vertex current = to;
        while (current != from) {
            path.push_back(current);
            current = predecessors[current];
        }
        path.push_back(from);
        reverse(path.begin(), path.end());

        Distance current_distance = 0;

        vector<Edge> result;
        current = path[0];
        size_t i = 0;
        while (current != to) {
            Vertex next = path.at(i);
            auto& edges = _data.find(current)->second;
            for (auto& edge : edges) {
                if (edge.to == _data.find(next)->first && edge.distance + current_distance == distances[next]) {
                    result.push_back(edge);
                    current_distance += edge.distance;
                }
            }
            current = next;
            ++i;
        }
        return result;
    }



    //обход
    std::vector<Vertex>  walk(const Vertex& start_vertex) const {
        if (!has_vertex(start_vertex)) return {};

        vector<Vertex> visited;
        queue<Vertex> queue;
        unordered_map<Vertex, bool> visited_ver;

        queue.push(start_vertex);
        visited_ver[start_vertex] = true;

        while (!queue.empty()) {
            Vertex current = queue.front();
            queue.pop();
            visited.push_back(current);

            const auto& edges = _data.at(current);
            for (const auto& edge : edges) {
                if (!visited_ver[edge.to]) {
                    queue.push(edge.to);
                    visited_ver[edge.to] = true;
                }
            }

        }
        return visited;

    }

    /*Vertex warehouse() {
        Vertex farthest_vertex;
        double max_average_distance = 0;

        std::vector<Vertex> vertices = this->vertices();

        for (const auto& vertex : vertices) {
            std::vector<Edge> edges = this->edges(vertex);

            if (edges.size() < 2)
                continue;

            double total_distance = 0;

            for (const auto& edge : edges) {
                total_distance += edge.distance;
            }

            double average_distance = total_distance / edges.size();

            if (average_distance > max_average_distance) {
                max_average_distance = average_distance;
                farthest_vertex = vertex;
            }
        }

        return farthest_vertex;
    }*/

    private:
        unordered_map<Vertex, vector<Edge>> _data;

};

int main() {
    Graph<int, double> graph;

    for (int i = 1; i <= 10; i++) {
        graph.add_vertex(i);
    }

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dist(1, 10);

    for (int i = 1; i <= 10; i++) {
        for (int j = 1; j <= 10; j++) {
            if (i != j) {
                double distance = dist(gen) / 10.0;
                graph.add_edge(i, j, distance);
            }
        }
    }

    cout << "Has vertex 8: " << graph.has_vertex(8) << endl;
    cout << "Has edge (1, 7): " << graph.has_edge(1, 7) << endl;

    graph.remove_vertex(6);
    cout << "Has vertex 6: " << graph.has_vertex(6) << endl;
    graph.remove_edge(1, 3);
    cout << "Has edge (1, 3): " << graph.has_edge(1, 3) << endl;

    vector<int> walk = graph.walk(1);
    cout << "Walk starting from vertex 1: ";
    for (const auto& vertex : walk) {
        cout << vertex << " ";
    }
    cout << std::endl;

    vector<Graph<int, double>::Edge> shortestPath = graph.shortest_path(1, 10);
    cout << "Shortest path from 1 to 10: ";
    for (const auto& edge : shortestPath) {
        cout << "(" << edge.from << ", " << edge.to << ") ";
    }
    cout << std::endl;

    /*int farthest_vertex = graph.warehouse();
    std::cout << "Farthest vertex from neighbors: " << farthest_vertex << std::endl;*/

    return 0;
}


