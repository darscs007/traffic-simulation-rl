#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <exception>

#include "simulationConfig.h"
#include "Simulation.h"
#include <chrono>
#include "traceFormats.h"

std::string outputFileName = std::string(PROJECT_PATH) + "/output/";
std::ofstream lev2(outputFileName+"level2/detailed.bin", std::ios::binary);
std::ofstream lev0(outputFileName+"level0/statistics.bin", std::ios::binary);
std::ofstream lev1(outputFileName+"level1/congestion.bin", std::ios::binary);
std::ofstream g(std::string(PROJECT_PATH)+"/profiling/timeProfilingNew.out");

int main() 
{
 const auto start = std::chrono::steady_clock::now();
  
  simulationConfig config;
 
  if(!validateAndReportConfig(config))
    return EXIT_FAILURE;
    
  
  
  Simulation sim(config,lev0,lev1,lev2);
  
  try{sim.readCity();}
  catch(const std::exception& error)
  {
   std::cerr << "City reading error: " << error.what() << '\n';
   return EXIT_FAILURE;
  }

  sim.setTime(0.0f,0.1f); // tstep must be lower than the time it takes for a vehicle to travel the length of the shortest road at its maximum speed
  sim.initializeShortestPaths();
  
  try {sim.initializeWeights(config.cityDirectory + "demand.csv");}
  catch(const std::exception& error)
  {
   std::cerr << "Weight data reading error: " << error.what() << '\n';
   return EXIT_FAILURE;
  }
  
  sim.configureTrafficLights();
  sim.initializeVehicles();
  sim.initializeBuffers();

  if (!lev2.is_open()) 
  {
    std::cerr << "Can't open detailed.bin\n";
    return 1;
  }

  if (!lev0.is_open()) 
  {
    std::cerr << "Can't open statistics.bin\n";
    return 1;
  }

  if (!lev1.is_open()) 
  {
    std::cerr << "Can't open congestion.bin\n";
    return 1;
  }
  
  traceHeader lev0Header={STATS_MAGIC,TRACE_VERSION,config.steps,0}; 
  traceHeader lev1Header={CONGESTION_MAGIC,TRACE_VERSION,config.steps, sim.getNoRoads()};
  if(config.detailedRendering) 
  {
   traceHeader lev2Header={VEHICLES_MAGIC,TRACE_VERSION,config.steps,0};
   lev2.write(reinterpret_cast<const char*>(&lev2Header), sizeof(lev2Header));
  }

  lev0.write(reinterpret_cast<const char*>(&lev0Header), sizeof(lev0Header));
  lev1.write(reinterpret_cast<const char*>(&lev1Header), sizeof(lev1Header));

  g << "Preconfiguration: " << std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() << '\n';

  for(int i=0; i<config.steps; i++)
  {
        sim.oneStep(i);
  }
  
  sim.showProfile(g);

  const auto end = std::chrono::steady_clock::now();
  g << "Runtime: "<< std::chrono::duration<double>(end - start).count()<< " seconds\n";
  return 0;
}
