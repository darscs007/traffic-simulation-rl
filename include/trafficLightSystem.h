#pragma once

#include <vector>
#include <cityTypes.h>
#include <vehicle.h>
#include <simulationConfig.h>
#include <deque>
#include <Graph.h>
#include <functional>

struct roadObservation
{
int detectedVehicles;
int capacity;

};

struct stateTL
{
int currentPhase, timeSincePhase;

int externalQueueSize;
std::vector<roadObservation> incomingRoads;
std::vector<roadObservation> outgoingRoads;

std::vector<std::vector<int>> roadChangeMatrix;

std::vector<bool> adjTLPhases;
std::vector<int> adjTLTimes;
};

struct controlDataRL
{
  std::vector<stateTL> states;
  std::vector<double> rewards;
  int step;
  bool firstStep;
  bool done;

};

using sendData =std::function<void(const controlDataRL&)>;

using getActions =std::function<std::vector<int>()>;

struct RoadAtTL
{
  int roadId;
  int greenSteps;
  float score;
  float availability;
  int count;
};

struct trafficLight
{
  int id; // is equal to the intersection id
  std::vector<RoadAtTL> roadsTL;
  int currentState; //position in the roadsTL vector
  int nextChangeStep; //time when the next change will occur
  std::vector<std::vector<int>> roadChangeMatrix;
  double localReward;
  int countLocalVehicles;
};

class trafficLightSystem
{
  private:
   const simulationConfig& config;
   std::vector<trafficLight> trafficLights;
   std::vector<int> incomingPhaseId;
   std::vector<int> outgoingPhaseId;
   std::vector<int> phaseStartStep;
   double globalReward;
   sendData send;
   getActions receive;

  public:

  explicit trafficLightSystem(const simulationConfig& config);

  void setRLCallbacks(sendData sender, getActions receiver);
  
  void configureTrafficLights(const Graph& city);

  void updateTrafficLights(int step, const Graph& city);

  bool isGreenFor(int intersectionId, int thisRoadId);

  void recordRoadChange(const vehicle& v, const line& currentRoad, const line& targetRoad, int step);

  void recordDestination(int intersectionId, int roadId);

  void recordExtRoadChange(const vehicle&v, const line& targetRoad, int step);

  void internalRoadScoring(const line& currentRoad, const std::vector<std::deque<int>>& trafficQueues,const std::vector<vehicle>& vehicles,int step);

  void externalRoadScoring(const node& currentIntersection, int queSize, double timeFromIntersection ,int step, int count);

  int getCurrentGreenRoad(int currentIntersectionId) const;

  stateTL presentTL(int step,int currentIntersectionId, const Graph& city); //roads

  void calculateGlobalRewards(double fMoving, double fStationary);

  void addToLocalReward(int initialIntersectionId, float fraction, int count);

  bool validateActions(const std::vector<int>& actions);

  void sendLastData(const Graph& city);

  void addToFirstCounts(int intersectionId, int count);
};