#include "Simulation.h"
#include "trafficConstraints.h"
#include <random>
#include <iostream>
#include <fstream>
#include <sstream>
#include "traceFormats.h"
#include <cstdint>
#include <chrono>

  enum class LeaderState
{
  NoLeader,
  AdvancedOnRoad,
  Blocked,
  ExitedRoad
};
  
  void Simulation::transferToRoad(vehicle &v, double ftime, int targetRoadId, bool trafficQueue, int sourceId) 
  {
    const line& targetRoad=city.getLine(targetRoadId);

    v.currentRoadId=targetRoad.id;
    v.currentIntersectionId=targetRoad.from;
                    
    if(trafficQueues[targetRoad.id].size() && vehicles[trafficQueues[targetRoad.id][trafficQueues[targetRoad.id].size()-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/targetRoad.lg <ftime * targetRoad.maxspeed / targetRoad.lg) 
    {
    v.positionOnRoad =  vehicles[trafficQueues[targetRoad.id][trafficQueues[targetRoad.id].size()-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/targetRoad.lg;   
    }
    else  
    v.positionOnRoad=ftime * targetRoad.maxspeed / targetRoad.lg;
                    
    trafficQueues[targetRoad.id].push_back(v.id);
    
   if(trafficQueue) trafficQueues[sourceId].pop_front();
   else {waitQueues[sourceId].pop_front(); v.isActive=1;}
  }

  int Simulation::chooseNextBestRoad(vehicle &v, int currentIntersectionId, int currentTargetId)
  {
    int nextBestRoadId=-1;
    float nextBestTime=traffic::INF;

    const std::vector<int>& adjRoads=city.getAdjRoads(currentIntersectionId);
    for(int i=0; i< adjRoads.size(); i++)
    {
    const line& possibleRoad=city.getLine(adjRoads[i]);
    if(currentTargetId == adjRoads[i]) continue;
    if(possibleRoad.to != v.destinationIntersectionId && city.getLine(city.getNextRoadBetween(possibleRoad.to,v.destinationIntersectionId)).to == currentIntersectionId) continue;
    bool isNotForbidden=1;
    for(int j=0; j<v.forbiddenRoads.size() && isNotForbidden; j++)
        if(v.forbiddenRoads[j] == possibleRoad.id) isNotForbidden=0;

    if(!isNotForbidden) continue;

    if(trafficQueues[possibleRoad.id].size() && vehicles[trafficQueues[possibleRoad.id].back()].positionOnRoad * possibleRoad.lg < config.CAR_LENGTH+config.SAFETY_GAP) continue;

    if((float)possibleRoad.lg/possibleRoad.maxspeed + city.getTimeBetween(possibleRoad.to, v.destinationIntersectionId) < nextBestTime)
    {
    nextBestRoadId=possibleRoad.id;
    nextBestTime=(float)possibleRoad.lg/possibleRoad.maxspeed + city.getTimeBetween(possibleRoad.to, v.destinationIntersectionId);
    }
    }
     
    return nextBestRoadId;
  }

  bool Simulation::updateImpatience(vehicle &v, float fraction)
  {
    v.currentImpatience+=std::clamp(fraction,0.0f, 1.0f);
    v.currentImpatience*=config.IMPATIENCE_MULTIPLIER;
    return v.currentImpatience > v.impatienceThreshold;
  }
  
  Simulation::Simulation(const simulationConfig& configValue, std::ofstream& lev0, std::ofstream& lev1, std::ofstream& lev2)
    : config(configValue), rng(config.seed), patienceRng(config.seed), tlmanager(config), lev0(lev0), lev1(lev1), lev2(lev2)
    {}

  void Simulation::configureTrafficLights()
  {
   tlmanager.configureTrafficLights(city);
   std::cout << "Traffic lights configured successfully\n";
  }

  void Simulation::readCity()
  {
    city.readIntersections( config.cityDirectory+ "intersections.csv");
    city.readRoads(config.cityDirectory + "roads.csv");
    std::cout << "City data read from CSV files successfully\n";

  }

  void Simulation::setTime(double ctime, double tstep)
  {
    this->ctime = ctime;
    this->tstep = tstep;
    std::cout << "Time set successfully\n";
  }

  void Simulation::initializeShortestPaths()
  {
    city.initializeDijkstraShortestPaths();
    std::cout << "Shortest paths initialized successfully\n";
  }

  void Simulation::initializeVehicles()
  {
    chooseOrigin = std::discrete_distribution<int>(originWeights.begin(), originWeights.end());
    chooseDestination=std::discrete_distribution<int>(destinationWeights.begin(),destinationWeights.end());
    impatienceTresh=std::normal_distribution<float>(config.MEAN_IMPATIENCE_THRESHOLD,config.PATIENCE_STDDEV);
    patienceRegen=std::normal_distribution<float>(config.MEAN_PATIENCE_REGENERATION,0.1f);
    waitQueues.resize(city.getNoIntersections());
    trafficQueues.resize(city.getNoRoads());
    vehicles.resize(config.maxcars);
    nextAllowedEntry.resize(city.getNoIntersections(), 0.0);

    for(int i=0; i<config.maxcars; i++)
    {
      vehicle v{};
      v.lastStepProcessed=-1;
      v.destinationIntersectionId=chooseDestination(rng);
      v.currentIntersectionId=chooseOrigin(rng);
      while(v.currentIntersectionId==v.destinationIntersectionId) v.currentIntersectionId=chooseOrigin(rng);
      v.startIntersectionId=v.currentIntersectionId;
      waitQueues[v.currentIntersectionId].push_back(i);

      const node& currentIntersection=city.getIntersection(v.currentIntersectionId);

      v.id=i;
      v.expectedExternalTime=(waitQueues[v.currentIntersectionId].size()-1) * (config.CAR_LENGTH + config.SAFETY_GAP) / currentIntersection.externalSpeed;
      v.initTime=0.0;
      
      v.impatienceThreshold=impatienceTresh(patienceRng);
      while(v.impatienceThreshold < config.MEAN_IMPATIENCE_THRESHOLD-30 || v.impatienceThreshold > config.MEAN_IMPATIENCE_THRESHOLD+30) v.impatienceThreshold=impatienceTresh(patienceRng);
      v.patienceRegen=patienceRegen(patienceRng);
      while(v.patienceRegen < 0.0f || v.patienceRegen > 1.0f) v.patienceRegen=patienceRegen(patienceRng);
      v.currentImpatience=0.0f;
      v.isImpatient=0;

      vehicles[i] = v;
    }
    
    if(config.isRL)
      for(int i=0; i<waitQueues.size(); i++)
        tlmanager.addToFirstCounts(i, waitQueues[i].size());

    std::cout << "Vehicles initalized\n";
  }

  void Simulation::reinitializeVehicle(int id) // and calculating trip statistics
  {
     vehicle &v=vehicles[id];
    
    if(v.initTime+v.expectedExternalTime != 0.0)
     {
      tripStats.noTrips++;
      tripStats.teIntTime+=city.getTimeBetween(v.startIntersectionId,v.destinationIntersectionId);
      tripStats.taIntTime+=v.endTime-v.actualSpawnTime;
      tripStats.sumIntDiv+= (v.endTime-v.actualSpawnTime)/city.getTimeBetween(v.startIntersectionId,v.destinationIntersectionId);
      tripStats.sumTotDiv+= (v.endTime-v.initTime)/(v.expectedExternalTime+city.getTimeBetween(v.startIntersectionId,v.destinationIntersectionId));
      tripStats.sumExtDiv+= (v.actualSpawnTime-v.initTime)/v.expectedExternalTime;
      tripStats.teExtTime+=v.expectedExternalTime; //total expected=  te, total actual = ta
      tripStats.taExtTime+=v.actualSpawnTime-v.initTime;
      }
      
      v.destinationIntersectionId=chooseDestination(rng);
      v.currentIntersectionId=chooseOrigin(rng);
      while(v.currentIntersectionId==v.destinationIntersectionId) v.currentIntersectionId=chooseOrigin(rng);
      v.startIntersectionId=v.currentIntersectionId;
      if(!waitQueues[v.currentIntersectionId].empty()) 
        v.expectedExternalTime=(waitQueues[v.currentIntersectionId].size()+1) *(config.CAR_LENGTH + config.SAFETY_GAP)/city.getIntersection(v.currentIntersectionId).externalSpeed ;
      else v.expectedExternalTime=tstep;
      
      v.initTime=ctime;
      if(!waitQueues[v.currentIntersectionId].size()) nextAllowedEntry[v.currentIntersectionId]=v.initTime+v.expectedExternalTime;
      
      v.lastStepProcessed=-1;
      v.positionOnRoad=v.isActive=v.endTime=v.isImpatient=v.actualSpawnTime=0.0;
      v.currentImpatience = 0.0f;
      v.forbiddenRoads.clear();
      
      waitQueues[v.currentIntersectionId].push_back(v.id);
  }

  bool Simulation::mustWait(const line& currentRoad, const std::vector<bool>& isWaiting)
  {
     if (!trafficQueues[currentRoad.id].size())
          return 0;

     if(!tlmanager.isGreenFor(currentRoad.to, currentRoad.id)) 
        return 0;
    
    const vehicle& v=vehicles[trafficQueues[currentRoad.id][0]];
    if((1.0f-v.positionOnRoad)*currentRoad.lg/currentRoad.maxspeed > tstep) 
        return 0;

    if (currentRoad.to == v.destinationIntersectionId) return 0;
    
    const line& targetRoad= city.getLine(city.getNextRoadBetween(currentRoad.to,v.destinationIntersectionId));
    if(targetRoad.id < currentRoad.id && !isWaiting[targetRoad.id]) 
        return 0;

    if(!trafficQueues[targetRoad.id].size()) 
        return 0;

    double ftime=tstep - (1.0f-v.positionOnRoad)*currentRoad.lg/currentRoad.maxspeed;
    if(ftime*targetRoad.maxspeed < vehicles[trafficQueues[targetRoad.id].back()].positionOnRoad * targetRoad.lg - config.SAFETY_GAP - config.CAR_LENGTH) 
        return 0;

    if(!tlmanager.isGreenFor(targetRoad.to,targetRoad.id) && targetRoad.lg - (trafficQueues[targetRoad.id].size()+1)*(config.CAR_LENGTH+config.SAFETY_GAP) < 0) 
        return 0;

    return 1;
  }

  void Simulation::updateIntRoad(int i, int step, const line& currentRoad,stepMovementStats& mstats, std::vector<int>& pendingReinitializations)
  {
        float leaderAdvance=-1;
        LeaderState leaderState = LeaderState::NoLeader;
        std::deque<int>& trafficQueue= trafficQueues[i];
        for(int j=0; j<trafficQueue.size(); j++)
        {
        bool changedPaths=0;    
        vehicle &v=vehicles[trafficQueue[j]];
        int initialIntersectionId=city.getLine(v.currentRoadId).to;

         if(v.lastStepProcessed==step) continue;
         
         float initPos=v.positionOnRoad; 
         bool didNotExit=1;
         
         v.lastStepProcessed=step;
         
         if(leaderState == LeaderState::NoLeader || leaderState == LeaderState::Blocked || leaderState == LeaderState::ExitedRoad)
         {
         
          leaderAdvance=v.positionOnRoad;

          if(leaderState == LeaderState::NoLeader || leaderState == LeaderState::ExitedRoad)  v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
          else  
        if(tstep * currentRoad.maxspeed / currentRoad.lg < vehicles[trafficQueue[j-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/currentRoad.lg-v.positionOnRoad )  
               {
                v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
                leaderState = LeaderState::AdvancedOnRoad;
               }
            else 
               {
                double ftime= tstep - (vehicles[trafficQueue[j-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/currentRoad.lg-v.positionOnRoad)*currentRoad.lg/currentRoad.maxspeed;
                
                v.positionOnRoad = vehicles[trafficQueue[j-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/currentRoad.lg; 
                leaderState= LeaderState::Blocked;
   
                if(tlmanager.isGreenFor(currentRoad.to,currentRoad.id) && (1-v.positionOnRoad)*currentRoad.lg <= config.vehicleDetectionRadius) 
                  updateImpatience(v,ftime/tstep);
               }

         leaderAdvance=v.positionOnRoad-leaderAdvance;
            
         if(v.positionOnRoad >=1)
            {
             if(currentRoad.to == v.destinationIntersectionId) 
              {
                   if(tlmanager.isGreenFor(currentRoad.to,currentRoad.id)) 
                    { 
                    
                    double ftime=(1-initPos) * currentRoad.lg/currentRoad.maxspeed;
                    mstats.fractionalMoving+=ftime/tstep;
                    mstats.moving++; //special case of stats calculation

                    v.endTime=ctime-(tstep-ftime);
                    didNotExit=0;
                    leaderState = LeaderState::ExitedRoad;
                    pendingReinitializations.push_back(trafficQueue[j]);
                    trafficQueue.pop_front();
                    j--;

                    tlmanager.recordDestination(v.destinationIntersectionId,v.currentRoadId);
                    }
                   else 
                   {
                    leaderState=LeaderState::Blocked;
                    v.positionOnRoad=1;
                   }
              } 
             else 
               {
                double ftime= (v.positionOnRoad - 1) * currentRoad.lg / currentRoad.maxspeed;
                leaderState=LeaderState::Blocked; // attemts reaching the next road
                if(tlmanager.isGreenFor(currentRoad.to,currentRoad.id))
                {
                const line& targetRoad= city.getLine(city.getNextRoadBetween(currentRoad.to,v.destinationIntersectionId)); 
                if(!trafficQueues[targetRoad.id].size() || vehicles[trafficQueues[targetRoad.id].back()].positionOnRoad * targetRoad.lg >= config.CAR_LENGTH+config.SAFETY_GAP)
                {
                    
                    transferToRoad(v,ftime,targetRoad.id,1,currentRoad.id);
                    leaderState=LeaderState::ExitedRoad;
                    j--;
                    
                    tlmanager.recordRoadChange(v,currentRoad,targetRoad,step);
                    
                 }
                 else if(updateImpatience(v,ftime/tstep)) // impatience processing and rerouting
                  {
                    v.isImpatient=1;
                    int nextBestRoadId= chooseNextBestRoad(v,currentRoad.to,targetRoad.id);
                    
                    if(nextBestRoadId != -1)
                    {
                     v.forbiddenRoads.push_back(nextBestRoadId); 
                     
                     transferToRoad(v,ftime,nextBestRoadId,1,currentRoad.id);

                    leaderState=LeaderState::ExitedRoad;
                    j--;    
                    changedPaths=1;
                    v.currentImpatience-=v.patienceRegen*v.currentImpatience;
                    const line& chosenRoad=city.getLine(nextBestRoadId);
                    
                    tlmanager.recordRoadChange(v,currentRoad,chosenRoad,step);
                    }
                   }
                   
                 
               }
               
               if(leaderState == LeaderState::Blocked) v.positionOnRoad=1; 

               }
            }

           }
           else v.positionOnRoad+=leaderAdvance; // moving freely, but not reaching the enf of the road 
           
         if(didNotExit)
         {
           const line& finalRoad=city.getLine(v.currentRoadId);
           double ftime;
           if(currentRoad.id != finalRoad.id)
             ftime= (1-initPos) * currentRoad.lg/currentRoad.maxspeed + v.positionOnRoad * finalRoad.lg/finalRoad.maxspeed;
            
           else
              ftime=(v.positionOnRoad-initPos) * currentRoad.lg/currentRoad.maxspeed;
            
           mstats.addVehicle(ftime/tstep,1);
           tlmanager.addToLocalReward(initialIntersectionId,ftime/tstep,1);

           if(ftime/tstep + config.correction >= 0.9)v.currentImpatience =std::max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction
         }   
          
          v.isImpatient = changedPaths || v.currentImpatience > v.impatienceThreshold;
         }
        
        tlmanager.internalRoadScoring(currentRoad,trafficQueues,vehicles,step);

  }

  void Simulation::oneStep(int step) 
  {
    const auto start = std::chrono::steady_clock::now();
    
    //adding a pair vector and a deque vector that will retain the precedence for parsing
    //std::vector<int> waitingPredecessors(city.getNoIntersections(),-1); more time efficient if there are numerous waiting roads, to be implemented
    std::vector<std::pair<int,int>> pairs;
    std::vector<std::deque<int>> lists;
    std::vector<bool> isWaiting;
    isWaiting.resize(city.getNoRoads(),0);

    stepMovementStats mstats(config);
    std::vector<int> pendingReinitializations;

    

    const auto updateTL = std::chrono::steady_clock::now();
    tlmanager.updateTrafficLights(step,city);
    prof.trafficLights+= std::chrono::steady_clock::now() - updateTL;

    const auto vehiclesT=std::chrono::steady_clock::now();
    for(int i=0; i<city.getNoRoads(); i++)
      {
        const line &currentRoad=city.getLine(i);
        //checking first vehicle/traffic light// next road for order errors
        if(mustWait(currentRoad,isWaiting))
        {
          pairs.push_back({ currentRoad.id,city.getLine(city.getNextRoadBetween(currentRoad.to,vehicles[trafficQueues[currentRoad.id][0]].destinationIntersectionId)).id });
          isWaiting[currentRoad.id]=1;
          continue;
        }

        updateIntRoad(i,step,currentRoad,mstats,pendingReinitializations);

      }

    //making queues of order
    for(int i=pairs.size()-1; i >=0; i--)
    {
      bool foundList=0;
      for(int j=0; j < lists.size(); j++)
        if(lists[j][0] ==pairs[i].second){lists[j].push_front(pairs[i].first); foundList=1;}
        else if(lists[j].back() == pairs[i].first){lists[j].push_back(pairs[i].second); foundList=1;}

        if(!foundList) lists.push_back({pairs[i].first,pairs[i].second});
    }
     
    //here parsing the remaining roads
    for(int i=0; i<lists.size(); i++)
      for(int j=lists[i].size()-2; j>=0; j--)
        updateIntRoad(lists[i][j],step, city.getLine(lists[i][j]),mstats,pendingReinitializations);
    
    for(int i=0; i<waitQueues.size(); i++)
        {
         bool canProcessNextVehicle=1; 
         int NoInitVehicles=waitQueues[i].size();
         double lastMovementTime=0; // the last initialized vehicle movement time; 
         double iNextEntry=nextAllowedEntry[i];
        
         while(canProcessNextVehicle && !waitQueues[i].empty())
         {
         double timeSpentMoving=0.0;
         bool changedPaths=0;
         canProcessNextVehicle=0;
         vehicle &v=vehicles[waitQueues[i].front()];  
         v.currentRoadId = city.getNextRoadBetween(v.currentIntersectionId,v.destinationIntersectionId); 
         const line& currentRoad = city.getLine(v.currentRoadId);
         const node& currentIntersection = city.getIntersection(v.currentIntersectionId);

        //if the vehicle has not yet left the waiting queue
        NoInitVehicles--;
        if(nextAllowedEntry[v.currentIntersectionId] < v.initTime+v.expectedExternalTime) 
        {
         nextAllowedEntry[v.currentIntersectionId] = v.initTime+v.expectedExternalTime;

        }
        if(ctime >= v.initTime+v.expectedExternalTime && ctime + config.correction >= nextAllowedEntry[v.currentIntersectionId] && tlmanager.isGreenFor(i,traffic::externalRoadId) && (!trafficQueues[currentRoad.id].size() || vehicles[trafficQueues[currentRoad.id].back()].positionOnRoad * currentRoad.lg + config.correction >= config.CAR_LENGTH+config.SAFETY_GAP))
                {
                 canProcessNextVehicle=1;   
                v.actualSpawnTime=nextAllowedEntry[v.currentIntersectionId];
                double ftime=ctime-nextAllowedEntry[v.currentIntersectionId];
                 
                transferToRoad(v,ftime, currentRoad.id,0,i);

                double moveTime=v.positionOnRoad*currentRoad.lg/currentRoad.maxspeed; //on internal Road
                timeSpentMoving=nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime;
                
                 lastMovementTime=1.0;
              
                 if((nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep+config.correction >= 0.9)v.currentImpatience =std::max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction   

                 nextAllowedEntry[v.currentIntersectionId]+= (config.CAR_LENGTH + config.SAFETY_GAP) / currentIntersection.externalSpeed;  
                 
                 tlmanager.recordExtRoadChange(v,currentRoad,step);
                }  
        else if(ctime >= v.initTime+v.expectedExternalTime && ctime + config.correction >= nextAllowedEntry[v.currentIntersectionId])
        {
                
              if(tlmanager.isGreenFor(i,traffic::externalRoadId))
              {
                lastMovementTime=(nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep;

                if(updateImpatience(v,1-(nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep)) // impatience processing and rerouting
                {
                    v.isImpatient=1;

                int nextBestRoadId=chooseNextBestRoad(v,v.currentIntersectionId,v.currentRoadId);
                  
                if(nextBestRoadId != -1)
                { 
                 lastMovementTime=1.0;
                 canProcessNextVehicle=1;   
                 v.isActive=1;
                 v.forbiddenRoads.push_back(nextBestRoadId);
                 v.currentImpatience -= v.patienceRegen * v.currentImpatience;
                 const line& targetRoad=city.getLine(nextBestRoadId); //redeclarare current Road
                v.actualSpawnTime=nextAllowedEntry[v.currentIntersectionId];
                double ftime=ctime-nextAllowedEntry[v.currentIntersectionId];

                transferToRoad(v,ftime,nextBestRoadId,0,v.currentIntersectionId);     

                 double moveTime=v.positionOnRoad*targetRoad.lg/targetRoad.maxspeed;
                 timeSpentMoving=nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime;

                 nextAllowedEntry[v.currentIntersectionId]+= (config.CAR_LENGTH + config.SAFETY_GAP) / currentIntersection.externalSpeed;  
                 changedPaths=1;
                 tlmanager.recordExtRoadChange(v,targetRoad,step);
                  }
                }
              
              if(!changedPaths)
              { 
              double firstOffset = std::max(0.0,nextAllowedEntry[i]-ctime) * city.getIntersection(i).externalSpeed ; //changed 

              for(int e=1; e < waitQueues[i].size(); e++)
                    if( 1.0f - firstOffset/ config.vehicleDetectionRadius > 0)  
                    {
                      updateImpatience(vehicles[waitQueues[i][e]], 1.0f - (nextAllowedEntry[i]-iNextEntry)/tstep);
                      firstOffset+=config.CAR_LENGTH+config.SAFETY_GAP;
                    }
                    else break;
              }


              }
          
            if(!canProcessNextVehicle) // the leader has not exited
            {
            if(nextAllowedEntry[v.currentIntersectionId] - iNextEntry >= config.correction) 
                { 
                timeSpentMoving=nextAllowedEntry[v.currentIntersectionId] - iNextEntry;
                lastMovementTime=timeSpentMoving/tstep;
                }
              
              nextAllowedEntry[v.currentIntersectionId]=ctime;   
             }
        }
        else
        {
            v.currentImpatience =std::max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction
            lastMovementTime=1.0;
            timeSpentMoving=tstep;
        }
        
    
        v.isImpatient =changedPaths || v.currentImpatience > v.impatienceThreshold;
        
        mstats.addVehicle(timeSpentMoving/tstep,1);
        tlmanager.addToLocalReward(i,timeSpentMoving/tstep,1);
      }
        //general cases
        mstats.addVehicle(lastMovementTime,NoInitVehicles);
        tlmanager.addToLocalReward(i,lastMovementTime,NoInitVehicles);

        tlmanager.externalRoadScoring(city.getIntersection(i),waitQueues[i].size(),nextAllowedEntry[i]-ctime,step,waitQueues[i].size());
      } 
  
    for(int i=0; i<pendingReinitializations.size(); i++)
       reinitializeVehicle(pendingReinitializations[i]);

    prof.vehicleUpdate+= std::chrono::steady_clock::now() - vehiclesT;
    
    const auto statsT = std::chrono::steady_clock::now();
   std::uint32_t nr=0;
    
    for(int i=0; i<trafficQueues.size(); i++)
         nr+=trafficQueues[i].size();

    tlmanager.calculateGlobalRewards(mstats.fractionalMoving, mstats.fractionalStationary);

   
  if(config.writeOutput){ 
    statsFormat stats{tripStats.noTrips, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mstats.moving, mstats.stationary, mstats.fractionalMoving, mstats.fractionalStationary, nr};
    if(tripStats.noTrips) 
    {
      stats.extAvg= tripStats.sumExtDiv/tripStats.noTrips; 
      stats.intAvg= tripStats.sumIntDiv/tripStats.noTrips; 
      stats.totAvg= tripStats.sumTotDiv/tripStats.noTrips;
      stats.extW= tripStats.taExtTime/tripStats.teExtTime;
      stats.intW= tripStats.taIntTime/tripStats.teIntTime;
      stats.totW= (tripStats.taExtTime+tripStats.taIntTime)/(tripStats.teExtTime+tripStats.teIntTime);
    }

    congBuffer.clear();
    vehicleBuffer.clear();
    phaseBuffer.clear();

    lev0.write(reinterpret_cast<const char*>(&stats), sizeof(stats));

    prof.statistics+= std::chrono::steady_clock::now() - statsT;

    auto const congT = std::chrono::steady_clock::now();
    for(int i=0; i<city.getNoRoads(); i++)
    {
      float occupancy= std::max(0.0f,(trafficQueues[i].size()*(config.CAR_LENGTH+config.SAFETY_GAP)-config.SAFETY_GAP))/city.getLine(i).lg;
      congBuffer.push_back(occupancy);

    }
    lev1.write(reinterpret_cast<const char*>(congBuffer.data()), congBuffer.size() * sizeof(float));

    prof.congestionOutput+= std::chrono::steady_clock::now() - congT;

    const auto detailT = std::chrono::steady_clock::now();
   if(config.detailedRendering)
   {

    for(int i=0; i< city.getNoIntersections(); i++)
        phaseBuffer.push_back(tlmanager.getCurrentGreenRoad(i));
   
      lev2.write(reinterpret_cast<const char*>(phaseBuffer.data()), phaseBuffer.size()*sizeof(std::uint32_t));
      lev2.write(reinterpret_cast<const char*>(&nr), sizeof(nr));

   
    for(int i=0; i<trafficQueues.size(); i++)
      for(int j=0; j<trafficQueues[i].size(); j++)
        {
            const vehicle& v=vehicles[trafficQueues[i][j]];
            detailedVehicle det={v.id, v.currentRoadId, std::clamp(v.positionOnRoad, 0.0f, 1.0f), v.isImpatient};
            vehicleBuffer.push_back(det);
        }
         lev2.write(reinterpret_cast<const char*>(vehicleBuffer.data()), vehicleBuffer.size()* sizeof(detailedVehicle));
    }

    prof.detailedOutput+=std::chrono::steady_clock::now()-detailT;
    prof.total+=std::chrono::steady_clock::now()-start;
  }

    if(step==config.steps-1)
       tlmanager.sendLastData(city);

    ctime+=tstep;    
    }
  
  void Simulation::initializeWeights(const std::string& fileName)
  {

        std::ifstream file(fileName);
        if(!file.is_open()) 
          throw std::runtime_error("Cannot open weights data: " + fileName);
        
        bool okDest, okOrig;
        okDest=okOrig=0;
        std::string csvLine, id, destination_weight, origin_weight;

        getline(file, csvLine); // antet: x,y,id

        int currentId=0;
        while (getline(file, csvLine)) {
            std::stringstream stream(csvLine);

            std::getline(stream, id, ',');
            std::getline(stream, destination_weight, ',');
            std::getline(stream, origin_weight, ',');

            if(stoi(id) != currentId) 
              throw std::invalid_argument("Id not in order: " + csvLine);

            if(stoi(destination_weight) < 0)
              throw std::invalid_argument("Negative destination weight: " + csvLine);

            if(stoi(origin_weight) < 0)
              throw std::invalid_argument("Negative origin weight: " + csvLine);

            if(stoi(origin_weight) > 0) okOrig=1;
            if(stoi(destination_weight) >0) okDest=1;
            
            originWeights.push_back(stoi(origin_weight));
            destinationWeights.push_back(stoi(destination_weight));
            currentId++;
        }
        
      if(currentId != city.getNoIntersections())
        throw std::invalid_argument("Weights count does not match no of intersections");

      if(!okOrig)
        throw std::invalid_argument("At least one origin weight must be non zero");

      if(!okDest)
        throw std::invalid_argument("At least one destination weight must be non zero");
    
     std::cout << "Weights initialized\n";
    } 
  
  void Simulation::showShortestPaths()
  {
    
    for(int i=0; i< city.getNoIntersections(); i++)
    {
        for(int j=0; j < city.getNoIntersections(); j++)
          std::cout << city.getNextRoadBetween(i,j) <<  " ";

     std::cout << '\n';

    }

  }

  int Simulation::getNoRoads() const
  {
    return city.getNoRoads();
  }

  void Simulation::showProfile(std::ofstream &g)
  {
    g << "vehicle: " << std::chrono::duration<double>(prof.vehicleUpdate).count() << '\n';
    g << "lights: " << std::chrono::duration<double>(prof.trafficLights).count() << '\n';
    g << "stats: " << std::chrono::duration<double>(prof.statistics).count() << '\n';
    g << "congestion: " << std::chrono::duration<double>(prof.congestionOutput).count() << '\n';
    g << "detailed: " << std::chrono::duration<double>(prof.detailedOutput).count() << '\n';
    g << "total: " << std::chrono::duration<double>(prof.total).count() << '\n';
  }

  void Simulation::initializeBuffers()
  {
    vehicleBuffer.reserve(config.maxcars);
    phaseBuffer.reserve(city.getNoIntersections());
    congBuffer.reserve(city.getNoRoads());

  }

void Simulation::setRLCallbacks(sendData sender, getActions receiver)
{
    tlmanager.setRLCallbacks(std::move(sender), std::move(receiver));
}
  
void Simulation::reset(int i)
{
 rng.seed(config.seed+i);
 patienceRng.seed(config.seed+i);

 chooseOrigin.reset();
 chooseDestination.reset();
 impatienceTresh.reset();
 patienceRegen.reset();

 tripStats={};
 nextAllowedEntry.clear();
 trafficQueues.clear();
 waitQueues.clear();
 vehicles.clear();
 setTime(0.0,0.1);
 prof={};
 vehicleBuffer.clear();
 phaseBuffer.clear();
 congBuffer.clear();

 tlmanager.configureTrafficLights(city);
 initializeVehicles(); 
}
