#pragma once

#include <vector>
#include <cityTypes.h>
#include <vehicle.h>
#include <simulationConfig.h>
#include <deque>
#include <Graph.h>

class Graph;

struct RoadAtTL
{
  int roadId;
  int greenSteps;
  float score;
  float availability;
};

struct trafficLight
{
  int id; // is equal to the intersection id
  std::vector<RoadAtTL> roadsTL;
  int currentState; //position in the roadsTL vector
  int nextChangeStep; //time when the next change will occur
  std::vector<std::vector<int>> roadChangeMatrix;
};

class trafficLightSystem
{
  private:
   const simulationConfig& config;
   std::vector<trafficLight> trafficLights;
   std::vector<int> incomingPhaseId;
   std::vector<int> outgoingPhaseId;

  public:

  explicit trafficLightSystem(const simulationConfig& config);
  
  void configureTrafficLights(const Graph& city);

  void updateTrafficLights(int step, const Graph& city);

  bool isGreenFor(int intersectionId, int thisRoadId);

  void recordRoadChange(const vehicle& v, const line& currentRoad, const line& targetRoad, int step);

  void recordDestination(int intersectionId, int roadId);

  void recordExtRoadChange(const vehicle&v, const line& targetRoad, int step);

  void internalRoadScoring(const line& currentRoad, const std::vector<std::deque<int>>& trafficQueues,const std::vector<vehicle>& vehicles,int step);

  void externalRoadScoring(const node& currentIntersection, int queSize,float timeFromIntersection ,int step);

  int getCurrentGreenRoad(int currentIntersectionId) const;
};