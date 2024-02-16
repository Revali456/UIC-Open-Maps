#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <list>
#include <algorithm>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
   struct EdgeNode {
        VertexT dest;
        WeightT weight;

        EdgeNode(VertexT d, WeightT w) : dest(d), weight(w) {}
    };

    std::map<VertexT, std::list<EdgeNode>> AdjList;
    std::vector<VertexT> Vertices;

      int _LookupVertex(VertexT v) const {
        int low = 0;
        int high = Vertices.size() - 1;

        while (low <= high) {
            int mid = low + (high - low) / 2;

            if (Vertices[mid] == v) {
                return mid;  // Found the vertex
            } else if (Vertices[mid] < v) {
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }

        // If get here, the vertex is not found
        return -1;
    }

 public:
    // Default constructor
    graph() = default;

    // Function to add a vertex
    bool addVertex(VertexT v) {
        // Binary search for vertex existence
        auto it = lower_bound(this->Vertices.begin(), this->Vertices.end(), v);

        if (it != this->Vertices.end() && *it == v) {
            return false; // Vertex already exists.
        }

        this->Vertices.insert(it, v);
        this->AdjList[v] = {}; // Initialize empty adjacency list for the new vertex.

        return true;
    }

  //
  // Destructor:
  //
  // Cleans up dynamically allocated memory, if any.
  //
  ~graph() {
    // Clean up any dynamically allocated memory if needed.
  }

  // Add copy constructor and operator= if you need to perform deep copies.

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return static_cast<int>(this->Vertices.size());
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;

    for (const auto& entry : this->AdjList) {
      count += entry.second.size();
    }

    return count;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns true.
  // If the vertices do not exist, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
    bool addEdge(VertexT from, VertexT to, WeightT weight) {
        auto fromIt = this->AdjList.find(from);
        auto toIt = this->AdjList.find(to);

        // Check if both vertices exist in the graph.
        if (fromIt == this->AdjList.end() || toIt == this->AdjList.end()) {
            return false; // Vertices not found.
        }

        auto& edges = fromIt->second;
        auto edgeIt = lower_bound(edges.begin(), edges.end(), to,
            [](const EdgeNode& edge, const VertexT& vertex) {
                return edge.dest < vertex;
            });

        // Check if the edge already exists.
        if (edgeIt != edges.end() && edgeIt->dest == to) {
            edgeIt->weight = weight; // Update weight.
        } else {
            edges.insert(edgeIt, EdgeNode(to, weight)); // Insert the edge.
        }

        return true;
    }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    auto fromIt = this->AdjList.find(from);

    if (fromIt == this->AdjList.end()) {
      return false; // Source vertex not found.
    }

    // Search for the destination vertex in the adjacency list of the source.
    for (const auto& edge : fromIt->second) {
      if (edge.dest == to) {
        weight = edge.weight;
        return true;
      }
    }

    return false; // Edge not found.
  }

//
// neighbors
//
// Returns a set containing the neighbors of v, i.e., all
// vertices that can be reached from v along one edge.
// Since a set is returned, the neighbors are returned in
// sorted order.
//
set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;

    auto it = lower_bound(this->Vertices.begin(), this->Vertices.end(), v);

    if (it == this->Vertices.end() || *it != v) {
        return S; // Vertex not found.
    }

    // Add neighbors to the set.
    int index = distance(this->Vertices.begin(), it);

    for (const auto& edge : this->AdjList.at(this->Vertices[index])) {
        S.insert(edge.dest);
    }

    return S;
}

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    return this->Vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    for (int i = 0; i < this->NumVertices(); ++i) {
      output << " " << i << ". " << this->Vertices[i] << endl;
    }

    output << endl;
    output << "**Edges:" << endl;

    for (const auto& entry : this->AdjList) {
      output << " From " << entry.first << ": ";

      for (const auto& edge : entry.second) {
        output << "(" << edge.dest << "," << edge.weight << ") ";
      }

      output << endl;
    }

    output << "**************************************************" << endl;
  }
};

