#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <exception>

#include "simulationConfig.h"
#include "Simulation.h"

std::string outputFileName = std::string(PROJECT_PATH) + "/output/output.out";

std::ofstream g(outputFileName);

int main() 
{

  simulationConfig config;
   
  if(!validateAndReportConfig(config))
    return EXIT_FAILURE;
    
  Simulation sim(config,g);
  
  try{sim.readCity();}
  catch(const std::exception& error)
  {
   std::cerr << "City reading error: " << error.what() << '\n';
   return EXIT_FAILURE;
  }

  sim.setTime(0.0f,0.1f); // tstep must be lower than the time it takes for a vehicle to travel the length of the shortest road at its maximum speed
  sim.initializeShortestPaths();
   
  try {sim.initializeWeights(std::string(PROJECT_PATH) + "/data/demand.csv");}
  catch(const std::exception& error)
  {
   std::cerr << "Weight data reading error: " << error.what() << '\n';
   return EXIT_FAILURE;
  }

  sim.configureTrafficLights();
  sim.initializeVehicles();

  if (!g.is_open()) 
  {
    std::cerr << "Can't open output.out\n";
    return 1;
  }
   
  g << config.steps << '\n';
  for(int i=0; i<config.steps; i++)
  {
        sim.oneStep(i);

  }
  
  return 0;
}
