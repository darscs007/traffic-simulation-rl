#pragma once

#include "trafficConstraints.h"
#include "simulationConfig.h"
#include "Graph.h"
#include "trafficLightSystem.h"
#include <random>
#include <algorithm>
#include <deque>
#include <string>
#include <fstream>
#include "vehicle.h"

struct stepMovementStats
  {
    const simulationConfig& config;
    int moving=0;
    int stationary=0;
    float fractionalMoving=0.0f;
    float fractionalStationary=0.0f;
  
    stepMovementStats(const simulationConfig& config)
      : config(config)
    {}

    void addVehicle(float movementFraction, int count)
    {
      if(movementFraction <= config.correction) stationary+=count;
      else moving+=count;

      if(movementFraction <= config.correction) fractionalStationary+=count;
      else if(movementFraction + config.correction >= 1.0f) fractionalMoving+=count;
      else
      {
      fractionalMoving+=movementFraction * count;
      fractionalStationary+= (1.0f - movementFraction) *count;
      }
    }
  };

struct TripStatistics
  {
  float taExtTime=0.0f;
  float teExtTime=0.0f;
  float taIntTime=0.0f;
  float teIntTime=0.0f; 
  float sumExtDiv=0.0f; 
  float sumIntDiv=0.0f; 
  float sumTotDiv=0.0f;
  int noTrips=0;
  };

class Simulation
{
 private:
  simulationConfig config;
  Graph city;
  std::vector<vehicle> vehicles;
  float ctime; // current time of the simulation
  float tstep; // time step for the simulation
  std::vector<int> originWeights,destinationWeights;
  std::discrete_distribution<int> chooseOrigin,chooseDestination;
  std::mt19937 rng; // generate random number from seed
  std::mt19937 patienceRng; 
  std::vector<std::deque<int>> waitQueues;
  std::vector<std::deque<int>> trafficQueues;
  std::vector<float> nextAllowedEntry; // vector to store the next allowed entry time for each intersection
  std::normal_distribution<float> impatienceTresh;
  std::normal_distribution<float> patienceRegen;
  trafficLightSystem tlmanager;
  TripStatistics tripStats;
  std::ofstream& g;

  void transferToRoad(vehicle &v, float ftime, int targetRoadId, bool trafficQueue, int sourceId);

  int chooseNextBestRoad(vehicle &v, int currentIntersectionId, int currentTargetId);
  
  bool updateImpatience(vehicle &v, float fraction);
  
 public:
  explicit Simulation(const simulationConfig& configValue, std::ofstream& gvalue);

  void configureTrafficLights();

  void readCity();

  void setTime(float ctime, float tstep);

  void initializeShortestPaths();

  void initializeVehicles();

  void reinitializeVehicle(int id); // and calculating trip statistics

  bool mustWait(const line& currentRoad, const std::vector<bool>& isWaiting);

  void updateIntRoad(int i, int step, const line& currentRoad,stepMovementStats& mstats, std::vector<int>& pendingReinitializations);
  
  void oneStep(int step);
  
  void initializeWeights(const std::string& fileName);

  void showShortestPaths();

};