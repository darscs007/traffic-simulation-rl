#pragma once

#include <string>
#include <vector>
#include <iostream>
struct simulationConfig
{
float CAR_LENGTH = 1.0f;
float SAFETY_GAP= 1.5f;
int steps= 4000;
int maxcars=6000;
double correction =1e-6;
int defaultGreenSteps =20;
int minGreenSteps =10;
int detectionRadius= 28;
bool isAdaptive=1;
int seed =100;
float IMPATIENCE_PROPAGATION =0.4f; // may be a feature in the future
int MEAN_IMPATIENCE_THRESHOLD =50;
float MEAN_PATIENCE_REGENERATION =0.5f;
int PATIENCE_STDDEV = 12;
float IMPATIENCE_MULTIPLIER =1.1f;
int vehicleDetectionRadius =20;
float fullStepImpatienceReduction =0.5f;

int minYellowJam=10;
bool detailedRendering=1;


std::vector<std::string> validate() const
{
   std::vector<std::string> errors; 
   if(CAR_LENGTH <= 0.0f) errors.push_back("Negative car length");
   if(SAFETY_GAP < 0.0f) errors.push_back("Negative safety gap");
   if(steps <= 0) errors.push_back("Non-positive step count");
   if(maxcars <= 0) errors.push_back("Non-positive vehicle count");
   if(correction <= 0.0 || correction >= 1.0) errors.push_back("Invalid correction value");
   if(defaultGreenSteps <= 0) errors.push_back("Non-positive default green steps");
   if(minGreenSteps <= 0) errors.push_back("Non-positive minimum green steps");
   if(minGreenSteps > defaultGreenSteps) errors.push_back("Minimum green steps exceed default green steps");
   if(detectionRadius <= 0) errors.push_back("Non-positive traffic light detection radius");
   if(vehicleDetectionRadius <= 0) errors.push_back("Non-positive vehicle detection radius");
   if(IMPATIENCE_PROPAGATION < 0.0f || IMPATIENCE_PROPAGATION > 1.0f) errors.push_back("Invalid impatience propagation factor");
   if(MEAN_IMPATIENCE_THRESHOLD <= 0) errors.push_back("Non-positive mean impatience threshold");
   if(MEAN_PATIENCE_REGENERATION < 0.0f || MEAN_PATIENCE_REGENERATION > 1.0f) errors.push_back("Invalid mean patience regeneration");
   if(PATIENCE_STDDEV < 0) errors.push_back("Negative patience standard deviation");
   if(IMPATIENCE_MULTIPLIER <= 0.0f) errors.push_back("Non-positive impatience multiplier");
   if(fullStepImpatienceReduction < 0.0f) errors.push_back("Negative full-step impatience reduction");

   return errors;
}


};

inline bool validateAndReportConfig(const simulationConfig& config)
{
 std::vector<std::string> errors= config.validate();

 if(!errors.size()) return 1;

 for(int i=0; i< errors.size(); i++)
      std::cout << errors[i] << '\n';
  return 0;
}