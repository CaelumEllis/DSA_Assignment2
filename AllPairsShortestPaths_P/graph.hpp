#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <iostream>
#include <fstream>
#include <utility>
#include <functional>
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <limits>

template <typename T>
class Graph {
 private:
  std::vector<std::unordered_map<int, T> > adjList {};
  int numVertices {};

 public:
  // empty graph with N vertices
  explicit Graph(int N);

  // construct graph from edge list in filename
  explicit Graph(const std::string& filename);

  // add an edge directed from vertex i to vertex j with given weight
  void addEdge(int i, int j, T weight);

  // removes edge from vertex i to vertex j
  void removeEdge(int i, int j);

  // is there an edge from vertex i to vertex j?
  bool isEdge(int i, int j) const;

  // return weight of edge from i to j
  // will throw an exception if there is no edge from i to j
  T getEdgeWeight(int i, int j) const;

  // returns number of vertices in the graph
  int size() const;

  // return iterator to a particular vertex
  const std::unordered_map<int, T>& neighbours(int a) const {
    return adjList.at(a);
  }
};

template <typename T>
Graph<T>::Graph(int N) : adjList(N), numVertices {N} {}

template <typename T>
Graph<T>::Graph(const std::string& inputFile) {
  std::ifstream infile {inputFile};
  if (!infile) {
    std::cerr << inputFile << " could not be opened\n";
    return;
  }
  // first line has number of vertices
  infile >> numVertices;
  adjList.resize(numVertices);
  int i {};
  int j {};
  double weight {};
  // assume each remaining line is of form
  // origin dest weight
  while (infile >> i >> j >> weight) {
    addEdge(i, j, static_cast<T>(weight));
  }
}

template <typename T>
int Graph<T>::size() const {
  return numVertices;
}

template <typename T>
void Graph<T>::addEdge(int i, int j, T weight) {
  if (i < 0 or i >= numVertices or j < 0 or j >= numVertices) {
    throw std::out_of_range("invalid vertex number");
  }
  adjList[i].insert({j, weight});
}

template <typename T>
void Graph<T>::removeEdge(int i, int j) {
  // check if i and j are valid
  if (i >= 0 && i < numVertices && j >= 0 && j < numVertices) {
    adjList[i].erase(j);
  }
}

template <typename T>
bool Graph<T>::isEdge(int i, int j) const {
  if (i >= 0 && i < numVertices && j >= 0 && j < numVertices) {
    return adjList.at(i).contains(j);
  }
  return false;
}

template <typename T>
T Graph<T>::getEdgeWeight(int i, int j) const {
  return adjList.at(i).at(j);
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const Graph<T>& G) {
  for (int i = 0; i < G.size(); ++i) {
    out << i << ':';
    for (const auto& [neighbour, weight] : G.neighbours(i)) {
      out << " (" << i << ", " << neighbour << ")[" << weight << ']';
    }
    out << '\n';
  }
  return out;
}

// APSP functions
// Use this function to return an "infinity" value
// appropriate for the type T
template <typename T>
T infinity() {
  if (std::numeric_limits<T>::has_infinity) {
    return std::numeric_limits<T>::infinity();
  } else {
    return std::numeric_limits<T>::max();
  }
}

//moved bellman ford to its own function as it is required twice
template <typename T>
std::pair<std::vector<T>, bool> bellmanFord(const Graph<T>& G, int source) {
  size_t numVertices = G.size();
  std::vector<T> distances(numVertices, infinity<T>());
  distances[source] = 0;
  bool isNegativeCycle = false;
  // relaxing edges (numVertices - 1) times
  for (size_t i = 0; i < numVertices - 1; ++i) {
    for (size_t u = 0; u < numVertices; ++u) {
      for (const auto& [v, weight] : G.neighbours(u)) {
        if (distances[u] != infinity<T>() && distances[u] + weight < distances[v]) {
          distances[v] = distances[u] + weight;
        }
      }
    }
  }
  // checking for negative weight cycle
  for (size_t u = 0; u < numVertices; ++u) {
    for (const auto& [v, weight] : G.neighbours(u)) {
      if (distances[u] != infinity<T>() && distances[u] + weight < distances[v]) {
        isNegativeCycle = true;
        break;
      }
    }
    if (isNegativeCycle) break;
  }

  return {distances, isNegativeCycle};
}

// implement an algorithm for determining if G
// has a negative weight cycle here
template <typename T>
bool existsNegativeCycle(const Graph<T>& G) {
  size_t numVertices = G.size();
  Graph<T> modifiedGraph(numVertices + 1);

  // add back original edges to the modified gr1aph
  for (size_t u = 0; u < numVertices; ++u) {
    for (const auto& [v, weight] : G.neighbours(u)) {
      modifiedGraph.addEdge(u, v, weight);
    }
    // add edge from new source to each vertex
    modifiedGraph.addEdge(numVertices, u, 0);
  }

  // running bellmanFord on modified graph
  auto result = bellmanFord(modifiedGraph, numVertices);
  bool isNegativeCycle = result.second;
  return isNegativeCycle;
}

// implement Johnson's APSP algorithm here
template <typename T>
std::vector<std::vector<T> >
johnsonAPSP(const Graph<T>& G) {
  size_t numVertices = G.size();
  const T INF = infinity<T>();

  // trying to avoid floating point overflow
  using Int = long long;
  //using this allows me to pass mediumJohnson
  constexpr Int INF_AS_INT = std::numeric_limits<Int>::max();

  // creating a modified graph with a new source vertex (index n)
  Graph<T> extendedGraph(numVertices + 1);
  for (size_t u = 0; u < numVertices; u++) {
    for (const auto& [v, weight] : G.neighbours(u)) {
      extendedGraph.addEdge(u, v, weight);
    }
    //adding 0 edge to from new source to each vertex
    extendedGraph.addEdge(numVertices, u, static_cast<T>(0));
  }

  // running Bellman-Ford from new vertices to find their potentials h(vertex)
  auto [rawPotential, hasNegativeCycle] = bellmanFord(extendedGraph, numVertices);
  //if there's a negative cycle, return empty distance matrix
  if (hasNegativeCycle) {
    return {};
  }

  // initialize h(vertex) with raw potentials
  std::vector<Int> h(numVertices);
  for (size_t i = 0; i < numVertices; ++i) {
    h[i] = rawPotential[i];
  }

  // apply new weights w'(u,v) = w(u,v) + h(u) - h(v)
  Graph<Int> newWeightsGraph(numVertices);
  for (size_t u = 0; u < numVertices; ++u) {
    for (const auto& [v, weight] : G.neighbours(u)) {
      if (h[u] != INF_AS_INT && h[v] != INF_AS_INT) {
        Int newWeight = weight + h[u] - h[v];
        newWeightsGraph.addEdge(u, v, newWeight);
      }
    }
  }

  std::vector<std::vector<T>> distance(numVertices, std::vector<T>(numVertices, INF));
  // dijkstra from every source, using the reWeighted graph
  for (size_t src = 0; src < numVertices; ++src) {
    std::vector<Int> dist(numVertices, INF_AS_INT);
    dist[src] = 0;

    using MinHeap = std::priority_queue<std::pair<Int, int>, std::vector<std::pair<Int, int>>, std::greater<>>;
    MinHeap minPQ;
    
    minPQ.emplace(0, src);

    std::vector<bool> visited(numVertices, false);

    while (!minPQ.empty()) {
      auto [currentDist, u] = minPQ.top();
      minPQ.pop();

      if (visited[u]) {
        continue;
      } else {
        visited[u] = true;
      }

      for (const auto& [v, w] : newWeightsGraph.neighbours(u)) {
        if (dist[u] != INF_AS_INT && dist[u] + w < dist[v]) {
          dist[v] = dist[u] + w;
          minPQ.emplace(dist[v], v);
        }
      }
    }

    // reversing the reweighting
    for (size_t v = 0; v < numVertices; ++v) {
      if (dist[v] != INF_AS_INT && h[src] != INF_AS_INT && h[v] != INF_AS_INT) {
        Int correctedDistance = dist[v] + h[v] - h[src];
        if (correctedDistance < INF_AS_INT && correctedDistance > std::numeric_limits<T>::lowest()) {
          distance[src][v] = correctedDistance;
        }
      }
    }
  }
  return distance;
}

// implement the Floyd-Warshall APSP algorithm here
template <typename T>
std::vector<std::vector<T> >
floydWarshallAPSP(const Graph<T>& G) {
  size_t numVertices = G.size();
  const T INF = infinity<T>();
  std::vector<std::vector<T>> distanceMatrix(numVertices, std::vector<T>(numVertices, INF));

  // initialising distance matrix with 0-diagonal and edge weights
  for (size_t i = 0; i < numVertices; ++i) {
    distanceMatrix[i][i] = static_cast<T>(0);
    for (const auto& [j, weight] : G.neighbours(i)) {
      // assign edge weight only if it's shorter than the existing value of the same edge
      if (weight < distanceMatrix[i][j]) {
        distanceMatrix[i][j] = weight;
      }
    }
  }

  // k is the index of intermediate vertex
  for (size_t k = 0; k < numVertices; ++k) {
    // i is the index of source vertex
    for (size_t i = 0; i < numVertices; ++i) {
      //if no path from i to k, continue
      if (distanceMatrix[i][k] == INF) continue;
      // j is the index of destination vertex
      for (size_t j = 0; j < numVertices; ++j) {
        if (distanceMatrix[k][j] == INF) continue; // no path from k to j
        // Check for overflow before doing addition
        T distanceThroughK = distanceMatrix[i][k] + distanceMatrix[k][j];
        if (distanceThroughK < distanceMatrix[i][j]) {
          distanceMatrix[i][j] = distanceThroughK;
        }
      }
    }
  }
  return distanceMatrix;
}

#endif      // GRAPH_HPP_
