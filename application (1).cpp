// application.cpp 
// Ahmik Ruben Nallamala

// This file controls the main application that finds the shortest distance between two points
// on a map and shows the exact distance between two points and the meeting point.
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <limits>
#include <set>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"



using namespace std;
using namespace tinyxml2;

  const double INF = numeric_limits<double>::max();

template <typename VertexT, typename WeightT>
struct DijkstraResult {
    vector<VertexT> path;
    map<VertexT, WeightT> distances;
};

template <typename VertexT, typename WeightT>
void DijkstraShortestPath(graph<VertexT, WeightT>& G, VertexT startNode, VertexT destnode, bool leftorRight, bool unreachable) {
    // Create a priority queue to store vertices and their distances
    priority_queue<pair<WeightT, VertexT>, vector<pair<WeightT, VertexT>>, greater<pair<WeightT, VertexT>>> unvisitedQueue;

    // Create a map to store distances from the start node
    map<VertexT, WeightT> distances;

    // Create a map to store the parent of each node in the shortest path
    map<VertexT, VertexT> predecessors;

    
    vector<VertexT> path;

    // Enqueue all vertices with initial distance as Infinity
    for (const auto& vertex : G.getVertices()) {
        distances[vertex] = numeric_limits<WeightT>::max();
        unvisitedQueue.push({distances[vertex], vertex});
    }

    // startNode has a distance of 0 from itself
    distances[startNode] = 0;

    while (!unvisitedQueue.empty()) {
        // Visit vertex with minimum distance from startNode
        VertexT currentV = unvisitedQueue.top().second;
        unvisitedQueue.pop();

        // Explore neighbors of the current vertex
        set<VertexT> neighbors = G.neighbors(currentV);
        for (const VertexT& adjV : neighbors) {
            WeightT edgeWeight;
            if (G.getWeight(currentV, adjV, edgeWeight)) {
                // Calculate alternative path distance
                WeightT alternativePathDistance = distances[currentV] + edgeWeight;

                // If a shorter path from startNode to adjV is found,
                // update adjV's distance and predecessor
                if (alternativePathDistance < distances[adjV]) {
                    distances[adjV] = alternativePathDistance;
                    predecessors[adjV] = currentV;
                    unvisitedQueue.push({distances[adjV], adjV});
                }
            }
        }
    }

    // Print the distances and predecessors or use them as needed
    // ...
    VertexT destNode = destnode;
    VertexT current = destNode;

    while (predecessors.find(current) != predecessors.end()) {
        path.push_back(current);
        current = predecessors[current];
    }

    // Add the start node to the path
    path.push_back(startNode);

    double distance1 = distances[destNode];

    if (distance1 >= INF && unreachable == true){
        cout << "Sorry, destination unreachable." << endl;

    }
    else if (distance1 >= INF && unreachable == false){
        return;
    }
    else{

    // Print the path in reverse order
    if (leftorRight){
        cout << "Person 1's distance to dest: " << distance1 << " miles" << endl;
        cout << "Path: ";
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        cout << *it;

        // Print -> unless it's the last element
        if (it != path.rend() - 1) {
            cout << "->";
        }
    }

    cout << endl;

    } else{
    cout << "Person 2's distance to dest: " << distance1 << " miles" << endl;
    cout << "Path: ";
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        cout << *it;

        // Print -> unless it's the last element
        if (it != path.rend() - 1) {
            cout << "->";
        }
    }

    cout << endl;
    }
    }

}

long long findNearestNodeToBuilding(const Coordinates& buildingCoords, const std::vector<FootwayInfo>& Footways, const std::map<long long, Coordinates>& Nodes) {
    double minDistance = std::numeric_limits<double>::infinity();
    long long nearestNode = -1;

    for (const auto& footway : Footways) {
        const std::vector<long long>& nodesOnFootway = footway.Nodes;

        for (long long node : nodesOnFootway) {
            // Get coordinates of the current node
            double nodeLat = Nodes.at(node).Lat;
            double nodeLon = Nodes.at(node).Lon;

            // Calculate distance between the node and the building
            double distance = distBetween2Points(buildingCoords.Lat, buildingCoords.Lon, nodeLat, nodeLon);

            // Update nearest node if the current node is closer
            if (distance < minDistance) {
                minDistance = distance;
                nearestNode = node;
            }
        }
    }

    // Convert the nearest node ID to a string
    return nearestNode;
}
string findNearestBuilding(const Coordinates& midpoint, const BuildingInfo& building1, const BuildingInfo& building2, const vector<BuildingInfo>& buildings) {
    double minDistance = INF;
    string nearestBuilding;

    // Loop through all buildings
    for (const BuildingInfo& building : buildings) {
        double distance = distBetween2Points(midpoint.Lat, midpoint.Lon, building.Coords.Lat, building.Coords.Lon);

        // Update the nearest building if a closer one is found
        if (distance < minDistance) {
            minDistance = distance;
            nearestBuilding = building.Fullname;
        }
    }

    return nearestBuilding;
}

void printNodeCoordinates(const string& nodeName, const vector<BuildingInfo>& buildings) {
    auto nodeIt = find_if(buildings.begin(), buildings.end(),
        [nodeName](const BuildingInfo& building) {
            return building.Fullname == nodeName;
        });

    if (nodeIt != buildings.end()) {
        cout << "(" << nodeIt->Coords.Lat << ", " << nodeIt->Coords.Lon << ")" << endl;
    } else {
        cout << "Details for the node '" << nodeName << "' not found." << endl;
    }
}



//
// Implement your standard application here
//
void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);
    bool buildingsfound = true;

    // Function to find a match for the query in the Buildings vector
    auto findBuilding = [&Buildings](const string& query) -> int {
        // Search through abbreviations first
        auto abbrevMatch = find_if(Buildings.begin(), Buildings.end(),
            [query](const BuildingInfo& building) {
                return building.Abbrev == query;
            });

        if (abbrevMatch != Buildings.end()) {
            return distance(Buildings.begin(), abbrevMatch);
        }

        // If no match found by abbreviation, search for partial names
        auto partialMatch = find_if(Buildings.begin(), Buildings.end(),
            [query](const BuildingInfo& building) {
                return building.Fullname.find(query) != string::npos;
            });

        if (partialMatch != Buildings.end()) {
            return distance(Buildings.begin(), partialMatch);
        }

        // No match found
        return -1;
    };

    // Find the index for person 1's building
    int person1Index = findBuilding(person1Building);

    // Find the index for person 2's building
    int person2Index = findBuilding(person2Building);


    // Check and print the result
    if (person1Index == -1) {
        cout << "Person 1's building not found." << endl;
        buildingsfound = false;
    }

    if (person2Index == -1) {
        cout << "Person 2's building not found." << endl;
        buildingsfound = false;
    }

    while (buildingsfound){
    BuildingInfo building1 = Buildings[person1Index];
    BuildingInfo building2 = Buildings[person2Index];
        // Calculate midpoints using the existing functions
        // Coordinates midpoint1 = centerBetween2Points(
        //     Buildings[person1Index].Coords.Lat, Buildings[person1Index].Coords.Lon,
        //     Buildings[person2Index].Coords.Lat, Buildings[person2Index].Coords.Lon);

        Coordinates midpoint2 = centerBetween2Points(
            Buildings[person1Index].Coords.Lat, Buildings[person1Index].Coords.Lon,
            Buildings[person2Index].Coords.Lat, Buildings[person2Index].Coords.Lon);

string nearestBuilding = findNearestBuilding(midpoint2, building1, building2, Buildings);

    int person3Index = findBuilding(nearestBuilding);


long long nearestP1Node = findNearestNodeToBuilding(Buildings[person1Index].Coords, Footways, Nodes);
long long nearestP2Node = findNearestNodeToBuilding(Buildings[person2Index].Coords, Footways, Nodes);
long long nearestDestNode = findNearestNodeToBuilding(Buildings[person3Index].Coords, Footways, Nodes);

        // Print the output
        cout << "Person 1's point:" << endl;
        cout << Buildings[person1Index].Fullname << endl;
        cout << "(" << Buildings[person1Index].Coords.Lat << ", " << Buildings[person1Index].Coords.Lon << ")" << endl;

        cout << "Person 2's point:" << endl;
        cout << Buildings[person2Index].Fullname << endl;
        cout << "(" << Buildings[person2Index].Coords.Lat << ", " << Buildings[person2Index].Coords.Lon << ")" << endl;

cout << "Destination Building:" << endl;
        cout << Buildings[person3Index].Fullname << endl;
        cout << "(" << Buildings[person3Index].Coords.Lat << ", " << Buildings[person3Index].Coords.Lon << ")" << endl;


cout << "Nearest P1 node:" << endl;
cout << nearestP1Node << endl;
cout << "(" << Nodes.at(nearestP1Node).Lat << ", " << Nodes.at(nearestP1Node).Lon << ")" << endl;

cout << "Nearest P2 node:" << endl;
cout << nearestP2Node << endl;
cout << "(" << Nodes.at(nearestP2Node).Lat << ", " << Nodes.at(nearestP2Node).Lon << ")" << endl;

cout << "Nearest destination node:" << endl;
cout << nearestDestNode << endl;
cout << "(" << Nodes.at(nearestDestNode).Lat << ", " << Nodes.at(nearestDestNode).Lon << ")" << endl;

    // printNearestNodesDetails(nearestP1Node, nearestP2Node, nearestDestNode, Nodes)

        //
        // TO DO: lookup buildings, find nearest start and dest nodes, find center
        // run Dijkstra's alg from each start, output distances and paths to destination:
        //

        // Print the result
// Find and print the shortest path for Person 1
bool leftorRight = true;
bool unreachable = true;
DijkstraShortestPath(G, nearestP1Node, nearestDestNode, leftorRight, unreachable);

// Find and print the shortest path for Person 2
leftorRight = false;
unreachable = false;
DijkstraShortestPath(G, nearestP2Node, nearestDestNode, leftorRight, unreachable);



buildingsfound = false;

    }
    
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

Coordinates centerBetween2Points(const Coordinates& coord1, const Coordinates& coord2) {
    // Implementation of the midpoint calculation (e.g., averaging coordinates)
    // ...

    // For demonstration purposes, using simple averaging
    Coordinates midpoint;
    midpoint.Lat = (coord1.Lat + coord2.Lat) / 2.0;
    midpoint.Lon = (coord1.Lon + coord2.Lon) / 2.0;

    return midpoint;
}





// Function to add bidirectional edges based on footways
void addEdgesBasedOnFootways(graph<long long, double>& G, const map<long long, Coordinates>& Nodes, const vector<FootwayInfo>& Footways) {
    for (const auto& footway : Footways) {
        const vector<long long>& nodesOnFootway = footway.Nodes;
        for (size_t i = 0; i < nodesOnFootway.size() - 1; ++i) {
            long long sourceNode = nodesOnFootway[i];
            long long targetNode = nodesOnFootway[i + 1];

            // Calculate distance between nodes
            double distance = distBetween2Points(Nodes.at(sourceNode).Lat, Nodes.at(sourceNode).Lon,
                                                  Nodes.at(targetNode).Lat, Nodes.at(targetNode).Lon);

            // Add bidirectional edges
            G.addEdge(sourceNode, targetNode, distance);
            G.addEdge(targetNode, sourceNode, distance);
        }
    }
}

int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  //
  // TO DO: build the graph, output stats:

    // Add Nodes as Vertices
    for (const auto& node : Nodes) {
        G.addVertex(node.first);
    }

    // Add Edges Based on Footways
    addEdgesBasedOnFootways(G, Nodes, Footways);




  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
