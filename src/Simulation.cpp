#include "Simulation.h"
#include "trafficConstraints.h"
#include <random>
#include <iostream>
#include <fstream>
#include <sstream>

  enum class LeaderState
{
  NoLeader,
  AdvancedOnRoad,
  Blocked,
  ExitedRoad
};

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

  
  void Simulation::transferToRoad(vehicle &v, float ftime, int targetRoadId, bool trafficQueue, int sourceId) 
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
  
  Simulation::Simulation(const simulationConfig& configValue, std::ofstream& gvalue)
    : config(configValue), rng(config.seed), patienceRng(config.seed), tlmanager(config), g(gvalue)
    {}

  void Simulation::configureTrafficLights()
  {
   tlmanager.configureTrafficLights(city);
   std::cout << "Traffic lights configured successfully\n";
  }

  void Simulation::readCity()
  {
    city.readIntersections(std::string(PROJECT_PATH) + "/data/intersections.csv");
    city.readRoads(std::string(PROJECT_PATH) + "/data/roads.csv");
    std::cout << "City data read from CSV files successfully\n";

  }

  void Simulation::setTime(float ctime, float tstep)
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
    nextAllowedEntry.resize(city.getNoIntersections(), 0.0f);

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
      v.initTime=0.0f;
      
      v.impatienceThreshold=impatienceTresh(patienceRng);
      while(v.impatienceThreshold < config.MEAN_IMPATIENCE_THRESHOLD-30 || v.impatienceThreshold > config.MEAN_IMPATIENCE_THRESHOLD+30) v.impatienceThreshold=impatienceTresh(patienceRng);
      v.patienceRegen=patienceRegen(patienceRng);
      while(v.patienceRegen < 0.0f || v.patienceRegen > 1.0f) v.patienceRegen=patienceRegen(patienceRng);
      v.currentImpatience=0.0f;
      v.isImpatient=0;

      vehicles[i] = v;
    }
    std::cout << "Vehicles initalized\n";
  }

  void Simulation::reinitializeVehicle(int id) // and calculating trip statistics
  {
     vehicle &v=vehicles[id];
    
    if(v.initTime+v.expectedExternalTime != 0.0f)
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
      v.positionOnRoad=v.isActive=v.endTime=v.isImpatient=v.actualSpawnTime=0;
      v.currentImpatience = 0.0f;
      v.forbiddenRoads.clear();
      
      waitQueues[v.currentIntersectionId].push_back(v.id);
  }

  void Simulation::oneStep(int step) 
  {
    stepMovementStats mstats(config);
    std::vector<int> pendingReinitializations;

    tlmanager.updateTrafficLights(step,city);

    for(int i=0; i<city.getNoRoads(); i++)
      {
        float leaderAdvance=-1;
        LeaderState leaderState = LeaderState::NoLeader;
        
        for(int j=0; j<trafficQueues[i].size(); j++)
        {
        bool changedPaths=0;    
        const line &currentRoad=city.getLine(i);
        vehicle &v=vehicles[trafficQueues[i][j]];

         if(v.lastStepProcessed==step) continue;
         
         float initPos=v.positionOnRoad; 
         bool didNotExit=1;
         
         v.lastStepProcessed=step;
         
         if(leaderState == LeaderState::NoLeader || leaderState == LeaderState::Blocked || leaderState == LeaderState::ExitedRoad)
         {
         
          leaderAdvance=v.positionOnRoad;

          if(leaderState == LeaderState::NoLeader || leaderState == LeaderState::ExitedRoad)  v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
          else  
        if(tstep * currentRoad.maxspeed / currentRoad.lg < vehicles[trafficQueues[i][j-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/currentRoad.lg-v.positionOnRoad )  
               {
                v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
                leaderState = LeaderState::AdvancedOnRoad;
               }
            else 
               {
                float ftime= tstep - (vehicles[trafficQueues[i][j-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/currentRoad.lg-v.positionOnRoad)*currentRoad.lg/currentRoad.maxspeed;
                
                v.positionOnRoad = vehicles[trafficQueues[i][j-1]].positionOnRoad-(config.CAR_LENGTH+config.SAFETY_GAP)/currentRoad.lg; 
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
                    
                    float ftime=(1-initPos) * currentRoad.lg/currentRoad.maxspeed;
                    mstats.fractionalMoving+=ftime/tstep;
                    mstats.moving++; //special case of stats calculation

                    v.endTime=ctime-(tstep-ftime);
                    didNotExit=0;
                    leaderState = LeaderState::ExitedRoad;
                    pendingReinitializations.push_back(trafficQueues[i][j]);
                    trafficQueues[i].pop_front();
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
                float ftime= (v.positionOnRoad - 1) * currentRoad.lg / currentRoad.maxspeed;
                leaderState=LeaderState::Blocked; // attemts reaching the next road
                if(tlmanager.isGreenFor(currentRoad.to,currentRoad.id))
                {
                const line& targetRoad= city.getLine(city.getNextRoadBetween(currentRoad.to,v.destinationIntersectionId)); ////redeclarare current road
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
           float ftime;
           if(currentRoad.id != finalRoad.id)
             ftime= (1-initPos) * currentRoad.lg/currentRoad.maxspeed + v.positionOnRoad * finalRoad.lg/finalRoad.maxspeed;
            
           else
              ftime=(v.positionOnRoad-initPos) * currentRoad.lg/currentRoad.maxspeed;
            
           mstats.addVehicle(ftime/tstep,1);
           if(ftime/tstep + config.correction >= 0.9f)v.currentImpatience =std::max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction
         }   
          
          v.isImpatient = changedPaths || v.currentImpatience > v.impatienceThreshold;
         }
        
        const line& currentRoad=city.getLine(i);
        tlmanager.internalRoadScoring(currentRoad,trafficQueues,vehicles,step);
      }
     

      for(int i=0; i<waitQueues.size(); i++)
        {
         bool canProcessNextVehicle=1; 
         int NoInitVehicles=waitQueues[i].size();
         float lastMovementTime=0; // the last initialized vehicle movement time; 
         float iNextEntry=nextAllowedEntry[i];
        
         while(canProcessNextVehicle && !waitQueues[i].empty())
         {
         float timeSpentMoving=0.0f;
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
                float ftime=ctime-nextAllowedEntry[v.currentIntersectionId];
                 
                transferToRoad(v,ftime, currentRoad.id,0,i);

                float moveTime=v.positionOnRoad*currentRoad.lg/currentRoad.maxspeed; //on internal Road
                timeSpentMoving=nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime;
                
                 lastMovementTime=1.0f;
              
                 if((nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep+config.correction >= 0.9f)v.currentImpatience =std::max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction   

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
                 lastMovementTime=1.0f;
                 canProcessNextVehicle=1;   
                 v.isActive=1;
                 v.forbiddenRoads.push_back(nextBestRoadId);
                 v.currentImpatience -= v.patienceRegen * v.currentImpatience;
                 const line& targetRoad=city.getLine(nextBestRoadId); //redeclarare current Road
                v.actualSpawnTime=nextAllowedEntry[v.currentIntersectionId];
                float ftime=ctime-nextAllowedEntry[v.currentIntersectionId];

                transferToRoad(v,ftime,nextBestRoadId,0,v.currentIntersectionId);     

                 float moveTime=v.positionOnRoad*targetRoad.lg/targetRoad.maxspeed;
                 timeSpentMoving=nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime;

                 nextAllowedEntry[v.currentIntersectionId]+= (config.CAR_LENGTH + config.SAFETY_GAP) / currentIntersection.externalSpeed;  
                 changedPaths=1;
                 tlmanager.recordExtRoadChange(v,targetRoad,step);
                  }
                }
              
              if(!changedPaths)
              { 
              float firstOffset = std::max(0.0f,nextAllowedEntry[i]-ctime) * city.getIntersection(i).externalSpeed ; //changed 

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
            lastMovementTime=1.0f;
            timeSpentMoving=tstep;
        }
        
    
        v.isImpatient =changedPaths || v.currentImpatience > v.impatienceThreshold;
        
        mstats.addVehicle(timeSpentMoving/tstep,1);
      }
        //general cases
        mstats.addVehicle(lastMovementTime,NoInitVehicles);
        
        tlmanager.externalRoadScoring(city.getIntersection(i),waitQueues[i].size(),nextAllowedEntry[i]-ctime,step);
      } 
  
    for(int i=0; i<pendingReinitializations.size(); i++)
       reinitializeVehicle(pendingReinitializations[i]);

   std::cout << step << '\n';    
    int nr=0;
    
    g << tripStats.noTrips << " ";
    if(tripStats.noTrips) g << tripStats.sumExtDiv/tripStats.noTrips << " " << tripStats.sumIntDiv/tripStats.noTrips << " " << tripStats.sumTotDiv/tripStats.noTrips << " " << tripStats.taExtTime/tripStats.teExtTime << " " << tripStats.taIntTime/tripStats.teIntTime << " " << (tripStats.taExtTime+tripStats.taIntTime)/(tripStats.teExtTime+tripStats.teIntTime) << '\n'; 
    else g << 0 << '\n';

    if(step)g << mstats.moving << " " << mstats.stationary << " " << mstats.fractionalMoving << " " << mstats.fractionalStationary << '\n';
    else g << 0 <<" " <<0<<  " " << 0 << " " << 0 << '\n';

    for(int i=0; i< city.getNoIntersections(); i++)
        g << tlmanager.getCurrentGreenRoad(i) << " ";
    g << '\n';
    
    for(int i=0; i<trafficQueues.size(); i++)
         nr+=trafficQueues[i].size();
    
    g << nr << '\n';
    
    for(int i=0; i<vehicles.size(); i++)
    if(vehicles[i].isActive)
        {
            const vehicle& v=vehicles[i];
            const line& r=city.getLine(v.currentRoadId);
  
            g << v.id << " " << v.currentIntersectionId << " " << r.to << " " << v.currentRoadId << " " << v.positionOnRoad << " " << v.isImpatient<<'\n';
        }
    
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
