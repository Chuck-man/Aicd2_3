#include <iostream>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <iterator>
#include <string>
#include <queue>
#include <limits>

using namespace std;

template<typename Vertex, typename Distance = double>

struct Edge
{
    Vertex from, to;
    Distance len;
};

template<typename Vertex, typename Distance = double>

class Graph {
    unordered_map<Vertex, vector<Edge>> _data;

public:
    struct Edge
    {
        Vertex from, to;
        Distance len;
    };

    Graph() {};

    ~Graph()
    {
        _data.clear();
    }

    bool presence_vertex(const Vertex& g) {
        if (_data.find(g) == _data.end()) return false;
        return true;
    }

    void add_vertex(const Vertex& g) {
        if (presence_vertex(g)) return;
        _data.insert({ g, {} });
    }

};

