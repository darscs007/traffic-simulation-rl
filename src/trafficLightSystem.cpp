#include "trafficLightSystem.h"
#include "Graph.h"
#include "trafficConstraints.h"
#include <algorithm>


  trafficLightSystem::trafficLightSystem(const simulationConfig& config)
    : config(config)
  {}
  
  void trafficLightSystem::configureTrafficLights(const Graph& city)
  {
    trafficLights.clear();
    incomingPhaseId.clear();
    outgoingPhaseId.clear();
    trafficLights.resize(city.getNoIntersections());
    
    if(config.isAdaptive)
    {
    incomingPhaseId.assign(city.getNoRoads(), -1);
    outgoingPhaseId.assign(city.getNoRoads(), -1);
    }
    
    for (int i = 0; i < city.getNoIntersections(); i++)
    {
        trafficLights[i].id = i;
        trafficLights[i].roadsTL.push_back({traffic::externalRoadId,config.defaultGreenSteps,0}); // adding the external road
        trafficLights[i].currentState = 0;
        trafficLights[i].nextChangeStep = trafficLights[i].roadsTL[0].greenSteps;
    }
    
    for(int i=0; i<city.getNoIntersections(); i++)
    {
        const std::vector<int>& adjRoads=city.getAdjRoads(i);
        int ind=1;
        for(int j=0; j<adjRoads.size(); j++)
        {
            const line& currentRoad=city.getLine(adjRoads[j]);
            trafficLights[currentRoad.to].roadsTL.push_back({currentRoad.id,config.defaultGreenSteps});
            if(config.isAdaptive)
            {
            incomingPhaseId[currentRoad.id]=trafficLights[currentRoad.to].roadsTL.size()-1;
            outgoingPhaseId[currentRoad.id]=ind++; //column number, 0 is the external road, so we start from 1
            }
        }
    }

    if(config.isAdaptive)
        for(int i=0; i<trafficLights.size(); i++)
            trafficLights[i].roadChangeMatrix.resize(trafficLights[i].roadsTL.size(), std::vector<int>(city.getAdjRoads(i).size()+1,0));
  }

  void trafficLightSystem::updateTrafficLights(int step, const Graph& city)
  {

    for(int i=0; i<trafficLights.size(); i++)
    {
        if(config.isAdaptive && step &&step % (trafficLights[i].roadsTL.size()*config.defaultGreenSteps) == 0)
        {
         float maxscore=0.0f, distance=0.0f;
         while(distance <= config.detectionRadius) 
         {
           maxscore+=1-distance/config.detectionRadius;
           distance+=config.CAR_LENGTH+config.SAFETY_GAP; 
         }

         float tscore=0.0f;

         for(int j=0; j<trafficLights[i].roadsTL.size(); j++)
         {
           int sumP=0; //calculate the number of vehicles that went through the intersection
           for(int e=0; e< trafficLights[i].roadChangeMatrix[j].size(); e++)
               sumP+=trafficLights[i].roadChangeMatrix[j][e];
            
           if(!sumP) trafficLights[i].roadsTL[j].availability=1;
           else
           {
            trafficLights[i].roadsTL[j].availability= (float)trafficLights[i].roadChangeMatrix[j][0]/sumP;
            for(int e=1; e < trafficLights[i].roadChangeMatrix[j].size(); e++)
            {
            int outgoingRoadId= city.getAdjRoads(i)[e-1];
            trafficLights[i].roadsTL[j].availability+= (float) trafficLights[i].roadChangeMatrix[j][e] /sumP * (0.25f + 0.75f * (1.0f - std::clamp(trafficLights[city.getLine(outgoingRoadId).to].roadsTL[incomingPhaseId[outgoingRoadId]].score/maxscore,0.0f,1.0f))); 
            }
           }
         
         tscore+=trafficLights[i].roadsTL[j].score*trafficLights[i].roadsTL[j].availability;
         }

         if(tscore-config.correction>=0)
         { 
         int totalSteps=0;
         int usedSteps=0;

         for(int j=0; j<trafficLights[i].roadsTL.size(); j++)
            {
             
             trafficLights[i].roadsTL[j].greenSteps=  config.minGreenSteps;
             usedSteps+=trafficLights[i].roadsTL[j].greenSteps;
             totalSteps+=trafficLights[i].roadsTL[j].greenSteps;
            }
         
         for(int j=0; j<trafficLights[i].roadsTL.size(); j++)
            {
             int extraSteps= trafficLights[i].roadsTL[j].score*trafficLights[i].roadsTL[j].availability/ tscore * (trafficLights[i].roadsTL.size() * config.defaultGreenSteps - usedSteps);
             trafficLights[i].roadsTL[j].greenSteps+= extraSteps ;
             totalSteps+=extraSteps;
            }
        
          totalSteps=trafficLights[i].roadsTL.size()*config.defaultGreenSteps - totalSteps;
         int ind=0;
         
        while(totalSteps)
        {
         trafficLights[i].roadsTL[ind].greenSteps++;
         totalSteps--;
         ind++;
         ind%=trafficLights[i].roadsTL.size();
        }
         }
         else
         {
            for(int j=0;j<trafficLights[i].roadsTL.size(); j++)
              trafficLights[i].roadsTL[j].greenSteps=config.defaultGreenSteps;
             
         }


        }
        
        if(step >= trafficLights[i].nextChangeStep)
        {
           trafficLights[i].currentState = (trafficLights[i].currentState+1) % trafficLights[i].roadsTL.size();
           trafficLights[i].nextChangeStep += trafficLights[i].roadsTL[trafficLights[i].currentState].greenSteps;
        }
    }

    if(config.isAdaptive && step % config.defaultGreenSteps==0) 
      for(int i=0; i<trafficLights.size(); i++)
        for (int e = 0; e < trafficLights[i].roadsTL.size(); e++)
            trafficLights[i].roadsTL[e].score= 0.0f;
  }

  bool trafficLightSystem::isGreenFor(int intersectionId, int thisRoadId)
  {
    const trafficLight& tl= trafficLights[intersectionId];
    if(tl.roadsTL[tl.currentState].roadId == thisRoadId) return 1;
    return 0;

  }

  void trafficLightSystem::recordRoadChange(const vehicle& v, const line& currentRoad, const line& targetRoad, int step)
  {
    if(config.isAdaptive)
    {
    trafficLight& tl=trafficLights[currentRoad.to];  
    tl.roadChangeMatrix[incomingPhaseId[currentRoad.id]][outgoingPhaseId[targetRoad.id]]++; 
                     
    if(targetRoad.id < currentRoad.id && (step+1) % config.defaultGreenSteps == 0)
    trafficLights[targetRoad.to].roadsTL[incomingPhaseId[targetRoad.id]].score += std::max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *targetRoad.lg/ config.detectionRadius);    
    } 
  }

  void trafficLightSystem::recordDestination(int intersectionId, int roadId)
  {
    if(config.isAdaptive)
    {
    trafficLight& tl=trafficLights[intersectionId];
    tl.roadChangeMatrix[incomingPhaseId[roadId]][0]++;
    }
  }

  void trafficLightSystem::recordExtRoadChange(const vehicle&v, const line& targetRoad, int step)
  {
    if(config.isAdaptive)
    {
      trafficLight& tl=trafficLights[targetRoad.from];  
      tl.roadChangeMatrix[0][outgoingPhaseId[targetRoad.id]]++;

      if((step+1) % config.defaultGreenSteps == 0)
      trafficLights[targetRoad.to].roadsTL[incomingPhaseId[targetRoad.id]].score += std::max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *targetRoad.lg/ config.detectionRadius);
    }    
  }

  void trafficLightSystem::internalRoadScoring(const line& currentRoad, const std::vector<std::deque<int>>& trafficQueues,const std::vector<vehicle>& vehicles,int step)
  {
  if(config.isAdaptive && (step+1) % config.defaultGreenSteps == 0)
  {  
    int ind;  
    for(int e=0; e<trafficLights[currentRoad.to].roadsTL.size(); e++)
      if(trafficLights[currentRoad.to].roadsTL[e].roadId==currentRoad.id) {ind=e;break;}
        
    for(int j=0; j< trafficQueues[currentRoad.id].size(); j++)
            {
             
             const vehicle& v=vehicles[trafficQueues[currentRoad.id][j]];   
             if(1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ config.detectionRadius + config.correction < 0) break;
             else trafficLights[currentRoad.to].roadsTL[ind].score += 1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ config.detectionRadius;
             
            }
          } 
  } 

  void trafficLightSystem::externalRoadScoring(const node& currentIntersection, int queSize,float timeFromIntersection ,int step)
  {
  if(config.isAdaptive && (step+1) % config.defaultGreenSteps == 0) 
  {
  float firstOffset = std::max(0.0f,timeFromIntersection)*currentIntersection.externalSpeed; 
  for(int e=0; e < queSize; e++)
  if( 1.0f - firstOffset/ config.detectionRadius >0 )  {trafficLights[currentIntersection.id].roadsTL[0].score +=  1.0f - firstOffset/ config.detectionRadius; firstOffset+=config.CAR_LENGTH+config.SAFETY_GAP;}
  else break;
  }

  }

  int trafficLightSystem::getCurrentGreenRoad(int currentIntersectionId) const
  {
    return trafficLights[currentIntersectionId].roadsTL[trafficLights[currentIntersectionId].currentState].roadId;
  }
