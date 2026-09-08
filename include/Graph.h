#pragma once

#include <vector>
#include <unordered_map>
#include "cityTypes.h"
#include <string>

class Graph
{
private:
    std::vector<node> nodes;
    std::vector<line> lines;
    std::unordered_map<int, int> nodeIndexMap; // Map to store node id to index mapping
    std::unordered_map<int, int> lineIndexMap; // Map to store line id to index mapping
    std::vector<std::vector<int>> adjList; // Adjacency list to store the graph structure
    std::vector<std::vector<float>> timeMatrix;
    std::vector<std::vector<int>> shortestPaths;

    std::vector<std::string> validateRoad(int fromi, int toi, int lgi, int maxspeedi, int idi) const;

    std::vector<std::string> validateIntersection(int externalSpeedi, int idi) const;
public:
    const line& getLine(int id) const;

    const node& getIntersection(int id) const;

    void addNode(float x, float y, int externalSpeed,int id);

    void addLine(int from, int to, int lg, int maxspeed, int id);

    void readIntersections(const std::string& fileName);

    void getShortestPath(std::vector<std::vector<int>> &a, std::vector<std::vector<int>> &b, int source, int destination, int &x);//calculates the successors;
    
    void readRoads(const std::string& fileName);

   void initializeDijkstraShortestPaths();
    
   int getNoIntersections()const;

   int getNoRoads()const;

   const std::vector<int>& getAdjRoads(int id) const; // number of roads that start in the intersection with this id

   void showTimeMatrix()const;

   float getTimeBetween(int i, int j)const;

   int getNextRoadBetween(int i, int j)const;
};