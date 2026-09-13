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
    double fractionalMoving=0.0;
    double fractionalStationary=0.0;
  
    stepMovementStats(const simulationConfig& config)
      : config(config)
    {}

    void addVehicle(double movementFraction, int count)
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
  double taExtTime=0.0;
  double teExtTime=0.0;
  double taIntTime=0.0;
  double teIntTime=0.0; 
  double sumExtDiv=0.0; 
  double sumIntDiv=0.0; 
  double sumTotDiv=0.0;
  int noTrips=0;
  };

class Simulation
{
 private:
  simulationConfig config;
  Graph city;
  std::vector<vehicle> vehicles;
  double ctime; // current time of the simulation
  double tstep; // time step for the simulation
  std::vector<int> originWeights,destinationWeights;
  std::discrete_distribution<int> chooseOrigin,chooseDestination;
  std::mt19937 rng; // generate random number from seed
  std::mt19937 patienceRng; 
  std::vector<std::deque<int>> waitQueues;
  std::vector<std::deque<int>> trafficQueues;
  std::vector<double> nextAllowedEntry; // vector to store the next allowed entry time for each intersection
  std::normal_distribution<float> impatienceTresh;
  std::normal_distribution<float> patienceRegen;
  trafficLightSystem tlmanager;
  TripStatistics tripStats;
  std::ofstream& lev0;
  std::ofstream& lev1;
  std::ofstream& lev2;

  void transferToRoad(vehicle &v, double ftime, int targetRoadId, bool trafficQueue, int sourceId);

  int chooseNextBestRoad(vehicle &v, int currentIntersectionId, int currentTargetId);
  
  bool updateImpatience(vehicle &v, float fraction);
  
 public:
  explicit Simulation(const simulationConfig& configValue, std::ofstream& lev0, std::ofstream& lev1, std::ofstream& lev2);

  void configureTrafficLights();

  void readCity();

  void setTime(double ctime, double tstep);

  void initializeShortestPaths();

  void initializeVehicles();

  void reinitializeVehicle(int id); // and calculating trip statistics

  bool mustWait(const line& currentRoad, const std::vector<bool>& isWaiting);

  void updateIntRoad(int i, int step, const line& currentRoad,stepMovementStats& mstats, std::vector<int>& pendingReinitializations);
  
  void oneStep(int step);
  
  void initializeWeights(const std::string& fileName);

  void showShortestPaths();

  int getNoRoads() const;

};