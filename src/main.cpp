#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <unordered_map>
#include <queue>
#include <deque>
#include <random>
#include <algorithm>
#include <iomanip>
#include "simulationConfig.h"

#define externalRoadId -2
#define INF 1e9

using namespace std;

string outputFileName = string(PROJECT_PATH) + "/output/output.out";
ofstream g(outputFileName);

bool validateAndReportConfig(const simulationConfig& config)
{
 vector<string> errors= config.validate();

 if(!errors.size()) return 1;

 for(int i=0; i< errors.size(); i++)
      cout << errors[i] << '\n';
  return 0;
}

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
  vector<RoadAtTL> roadsTL;
  int currentState; //position in the roadsTL vector
  int nextChangeStep; //time when the next change will occur
  vector<vector<int>> roadChangeMatrix;
};

struct node //intersection
{
 float x,y;
 int externalSpeed;//the speed of vehicles that have this intersection as its origin
 int id;
 
};

struct line // road
{
 int from, to;
 int lg ,maxspeed, id;
};

struct vehicle
{
    int id;
    int startIntersectionId;
    int currentIntersectionId;
    int destinationIntersectionId;
    int currentRoadId;
    float positionOnRoad; // Position on the road as a float between 0.0 and 1.0
    bool isActive;
    int lastStepProcessed;
    float initTime, actualSpawnTime, endTime,expectedExternalTime;
    
    //speed is the road's maxspeed
    
    float impatienceThreshold;
    float currentImpatience;
    float patienceRegen;
    vector<int> forbiddenRoads; //rerouting to these roads took it to the same intersection, so it won't make the same mistakes
    bool isImpatient;
};

enum class LeaderState
{
  NoLeader,
  AdvancedOnRoad,
  Blocked,
  ExitedRoad
};

class Graph
{
private:
    vector<node> nodes;
    vector<line> lines;
    unordered_map<int, int> nodeIndexMap; // Map to store node id to index mapping
    unordered_map<int, int> lineIndexMap; // Map to store line id to index mapping
    vector<vector<int>> adjList; // Adjacency list to store the graph structure
    vector<vector<float>> timeMatrix;
    vector<vector<int>> shortestPaths;

    vector<string> validateRoad(int fromi, int toi, int lgi, int maxspeedi, int idi) const
    {
      vector<string> errors;

      if(nodeIndexMap.find(fromi) == nodeIndexMap.end()) 
          errors.push_back( "Invalid 'from' intersection id: " + to_string(fromi));
               
      if(nodeIndexMap.find(toi) == nodeIndexMap.end()) 
          errors.push_back("Invalid 'to' intersection id: " + to_string(toi));
       
      if(fromi == toi) 
           errors.push_back("Road cannot connect the same intersection: " + to_string(fromi));

      if(lgi <= 0) 
           errors.push_back("Invalid road length: " + to_string(lgi)); 

      if(maxspeedi <= 0) 
        errors.push_back("Invalid road max speed: " + to_string(maxspeedi));  
                   
      if(idi < 0) 
        errors.push_back("Invalid road id: " + to_string(idi));  

      if(lineIndexMap.find(idi) != lineIndexMap.end()) 
          errors.push_back("Duplicate road id found: " + to_string(idi)); 

      if(idi != lines.size())
      errors.push_back("Unconsecutive Road Id: "+ to_string(idi));

      return errors;
    }

    vector<string> validateIntersection(int externalSpeedi, int idi) const
    {
      vector<string> errors;
      
      if (nodeIndexMap.find(idi) != nodeIndexMap.end()) 
                errors.push_back("Duplicate intersection id found: " + to_string(idi));

      if(externalSpeedi <= 0)
      errors.push_back("Negative or zero external road speed: "+ to_string(externalSpeedi));
       
      if(idi != nodes.size())
      errors.push_back("Unconsecutive Intersection Id: "+ to_string(idi));

      return errors;
    }
public:
    const line& getLine(int id) const
    {
        return lines[id];

    }

    const node& getIntersection(int id) const
    {
      return nodes[id];

    }

    void addNode(float x, float y, int externalSpeed,int id)
       {
        nodes.push_back({x,y,externalSpeed,id});
        nodeIndexMap[id] = nodes.size()-1; // store the position of the intersection in the vector
        if(adjList.size() <= id)
        {
            adjList.resize(id+1);
        }
       }

    void addLine(int from, int to, int lg, int maxspeed, int id)
    { 
        lines.push_back({from, to, lg, maxspeed, id});
        lineIndexMap[id] = lines.size()-1; // store the position of the road in the vector
        adjList[from].push_back(lines.size()-1); // add vector index of the road in the adjacency list to know all the details of the roads that start from an intersection
    }

    void readIntersections(const string& fileName)
    {
        ifstream file(fileName);
        if(!file.is_open())
          throw runtime_error("Cannot open intersection data: " + fileName);
        
        string csvLine, x, y, externalSpeed,id;

        getline(file, csvLine); // antet: x,y,extspeed,id

        while (getline(file, csvLine)) {
            stringstream stream(csvLine);

            getline(stream, x, ',');
            getline(stream, y, ',');
            getline(stream,externalSpeed,',');
            getline(stream, id, ',');

            vector<string> errors=validateIntersection(stoi(externalSpeed), stoi(id));

            if(errors.size())
              throw invalid_argument("Invalid intersection entry: " + csvLine + " - "+ errors[0]);

            addNode(stof(x), stof(y), stoi(externalSpeed),stoi(id));
        }
    }

    void getShortestPath(vector<vector<int>> &a, vector<vector<int>> &b, int source, int destination, int &x)//calculates the successors
    {
      if(source == destination) return;

      if(b[source][destination] != -1) 
        {
         x=b[source][destination]; 
         return;
        }
        
        
      if(lines[a[source][destination]].from==source)  x=a[source][destination];
      else getShortestPath(a,b,source, lines[a[source][destination]].from,x);
         
      b[source][destination]=x;

    }
    
   void readRoads(const string& fileName)
    {
        ifstream file(fileName);
        if(!file.is_open())
         throw runtime_error("Cannot open road data: " + fileName);
        
        string csvLine, from, to, lg, maxspeed, id;

        getline(file, csvLine); // antet: from,to,lg,maxspeed,id

        while (getline(file, csvLine)) {
            stringstream stream(csvLine);

            getline(stream, from, ',');
            getline(stream, to, ',');
            getline(stream, lg, ',');
            getline(stream, maxspeed, ',');
            getline(stream, id, ',');
            int fromi=stoi(from);
            int toi=stoi(to);
            int lgi=stoi(lg);
            int maxspeedi=stoi(maxspeed);
            int idi=stoi(id);

            vector<string> errors= validateRoad(fromi,toi,lgi,maxspeedi,idi);
            if(errors.size())
              throw invalid_argument("Invalid road entry: " + csvLine + " - " + errors[0]);

            addLine(fromi, toi, lgi, maxspeedi, idi);
        }
    }

   void initializeDijkstraShortestPaths()
   {
    
    shortestPaths=vector<vector<int>> (nodes.size(), vector<int>(nodes.size(), -1));
    vector<vector<int>> successors(nodes.size(),vector<int>(nodes.size(), -1));

   //dijkstra O(n^2) for each node, total O(n^3). I will optimize it later with a priority queue O(n^2 log n)
    for(int i=0;i<nodes.size();i++)
    {

        vector<int> f(nodes.size(), 0);
        vector<float> d(nodes.size(), INF);

      
        d[i]=0;
        f[i]=1;

       for(int j=0; j<adjList[i].size(); j++)
            {
            const line& cline=lines[adjList[i][j]];
            d[cline.to]= (float)cline.lg / cline.maxspeed;
            shortestPaths[i][cline.to] = adjList[i][j];    
            }
       
        
        int ok=1;
        while(ok)
        {
         float mn=INF; int x;
         ok=0;
         for(int j=0; j < nodes.size(); j++)
            {
             if(f[j]==0 && d[j]<mn)
                    {mn=d[j]; x=j;ok=1;}
               
            }
         
         if(ok)
         {
          f[x]=1;
          for(int j=0; j<adjList[x].size(); j++)
           {
            const line& cline=lines[adjList[x][j]];
            if(!f[cline.to] && d[cline.to] > d[x] + (float)cline.lg/cline.maxspeed)
                {d[cline.to] = d[x] + (float)cline.lg/cline.maxspeed;
                 shortestPaths[i][cline.to] = adjList[x][j];
                }
           }
         }
         
        }

    }
    
    timeMatrix.resize(shortestPaths.size(),vector<float>(shortestPaths[0].size(),0)); // generate timeMatrix

    for(int i=0; i<timeMatrix.size(); i++)
        for(int j=0; j<timeMatrix[i].size(); j++)
           if(i!=j && !timeMatrix[i][j])
            {
             float toAdd=0;
             int dest=j;
             while(i!=dest)
             {
               const line& currentRoad= getLine(shortestPaths[i][dest]);
               toAdd+=(float) currentRoad.lg/ currentRoad.maxspeed;

               timeMatrix[currentRoad.from][j]=toAdd;
               dest=currentRoad.from;
             }
             
            }
    
    int x;
    
    for(int i=0; i<nodes.size();i++)
        for(int j=0; j<nodes.size(); j++)
               getShortestPath(shortestPaths,successors,i,j,x);

    shortestPaths=successors;
   }
    
   int getNoIntersections()const
   {
    return nodes.size();
   }

   int getNoRoads()const
   {
    return lines.size();
   }

   const vector<int>& getAdjRoads(int id) const // number of roads that start in the intersection with this id
   {
    return adjList[id];
   }

   void showTimeMatrix()const
   {
    for(int i=0; i<timeMatrix.size(); i++)
        {for(int j=0; j<timeMatrix[i].size(); j++) cout << timeMatrix[i][j] << " ";
         cout << '\n';
        }
   }

   float getTimeBetween(int i, int j)const
   {
    return timeMatrix[i][j];
   }

   int getNextRoadBetween(int i, int j)const
   {
    return shortestPaths[i][j];
   }
};

class trafficLightSystem
{
  private:
   const simulationConfig& config;
   vector<trafficLight> trafficLights;
   vector<int> incomingPhaseId;
   vector<int> outgoingPhaseId;

  public:

  explicit trafficLightSystem(const simulationConfig& config)
    : config(config)
  {}
  
  void configureTrafficLights(const Graph& city)
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
        trafficLights[i].roadsTL.push_back({externalRoadId,config.defaultGreenSteps,0}); // adding the external road
        trafficLights[i].currentState = 0;
        trafficLights[i].nextChangeStep = trafficLights[i].roadsTL[0].greenSteps;
    }
    
    for(int i=0; i<city.getNoIntersections(); i++)
    {
        const vector<int>& adjRoads=city.getAdjRoads(i);
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
            trafficLights[i].roadChangeMatrix.resize(trafficLights[i].roadsTL.size(), vector<int>(city.getAdjRoads(i).size()+1,0));
  }

  void updateTrafficLights(int step, const Graph& city)
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
            trafficLights[i].roadsTL[j].availability+= (float) trafficLights[i].roadChangeMatrix[j][e] /sumP * (0.25f + 0.75f * (1.0f - clamp(trafficLights[city.getLine(outgoingRoadId).to].roadsTL[incomingPhaseId[outgoingRoadId]].score/maxscore,0.0f,1.0f))); 
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

  bool isGreenFor(int intersectionId, int thisRoadId)
  {
    const trafficLight& tl= trafficLights[intersectionId];
    if(tl.roadsTL[tl.currentState].roadId == thisRoadId) return 1;
    return 0;

  }

  void recordRoadChange(const vehicle& v, const line& currentRoad, const line& targetRoad, int step)
  {
    if(config.isAdaptive)
    {
    trafficLight& tl=trafficLights[currentRoad.to];  
    tl.roadChangeMatrix[incomingPhaseId[currentRoad.id]][outgoingPhaseId[targetRoad.id]]++; 
                     
    if(targetRoad.id < currentRoad.id && (step+1) % config.defaultGreenSteps == 0)
    trafficLights[targetRoad.to].roadsTL[incomingPhaseId[targetRoad.id]].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *targetRoad.lg/ config.detectionRadius);    
    } 
  }

  void recordDestination(int intersectionId, int roadId)
  {
    if(config.isAdaptive)
    {
    trafficLight& tl=trafficLights[intersectionId];
    tl.roadChangeMatrix[incomingPhaseId[roadId]][0]++;
    }
  }

  void recordExtRoadChange(const vehicle&v, const line& targetRoad, int step)
  {
    if(config.isAdaptive)
    {
      trafficLight& tl=trafficLights[targetRoad.from];  
      tl.roadChangeMatrix[0][outgoingPhaseId[targetRoad.id]]++;

      if((step+1) % config.defaultGreenSteps == 0)
      trafficLights[targetRoad.to].roadsTL[incomingPhaseId[targetRoad.id]].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *targetRoad.lg/ config.detectionRadius);
    }    
  }

  void internalRoadScoring(const line& currentRoad, const vector<deque<int>>& trafficQueues,const vector<vehicle>& vehicles,int step)
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

  void externalRoadScoring(const node& currentIntersection, int queSize,float timeFromIntersection ,int step)
  {
  if(config.isAdaptive && (step+1) % config.defaultGreenSteps == 0) 
  {
  float firstOffset = max(0.0f,timeFromIntersection)*currentIntersection.externalSpeed; 
  for(int e=0; e < queSize; e++)
  if( 1.0f - firstOffset/ config.detectionRadius >0 )  {trafficLights[currentIntersection.id].roadsTL[0].score +=  1.0f - firstOffset/ config.detectionRadius; firstOffset+=config.CAR_LENGTH+config.SAFETY_GAP;}
  else break;
  }

  }

  int getCurrentGreenRoad(int currentIntersectionId) const
  {
    return trafficLights[currentIntersectionId].roadsTL[trafficLights[currentIntersectionId].currentState].roadId;
  }
};

class Simulation
{
 private:
  simulationConfig config;
  Graph city;
  vector<vehicle> vehicles;
  float ctime; // current time of the simulation
  float tstep; // time step for the simulation
  vector<int> originWeights,destinationWeights;
  discrete_distribution<int> chooseOrigin,chooseDestination;
  mt19937 rng; // generate random number from seed
  mt19937 patienceRng; 
  vector<deque<int>> waitQueues;
  vector<deque<int>> trafficQueues;
  vector<float> nextAllowedEntry; // vector to store the next allowed entry time for each intersection
  normal_distribution<float> impatienceTresh;
  normal_distribution<float> patienceRegen;
  trafficLightSystem tlmanager;
  struct stepMovementStats
  {
    const simulationConfig& config;
    int moving=0;
    int stationary=0;
    float fractionalMoving=0.0f;
    float fractionalStationary=0.0f;
  
    explicit stepMovementStats(const simulationConfig& config)
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
  TripStatistics tripStats;
  
  void transferToRoad(vehicle &v, float ftime, int targetRoadId, bool trafficQueue, int sourceId) 
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

  int chooseNextBestRoad(vehicle &v, int currentIntersectionId, int currentTargetId)
  {
    int nextBestRoadId=-1;
    float nextBestTime=INF;

    const vector<int>& adjRoads=city.getAdjRoads(currentIntersectionId);
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

  bool updateImpatience(vehicle &v, float fraction)
  {
    v.currentImpatience+=clamp(fraction,0.0f, 1.0f);
    v.currentImpatience*=config.IMPATIENCE_MULTIPLIER;
    return v.currentImpatience > v.impatienceThreshold;
  }
  
 public:
  explicit Simulation(const simulationConfig& configValue)
    : config(configValue), rng(config.seed), patienceRng(config.seed), tlmanager(config)
    {}

  void configureTrafficLights()
  {
   tlmanager.configureTrafficLights(city);
   cout << "Traffic lights configured successfully\n";
  }

  void readCity()
  {
    city.readIntersections(string(PROJECT_PATH) + "/data/intersections.csv");
    city.readRoads(string(PROJECT_PATH) + "/data/roads.csv");
    cout << "City data read from CSV files successfully\n";

  }

  void setTime(float ctime, float tstep)
  {
    this->ctime = ctime;
    this->tstep = tstep;
    cout << "Time set successfully\n";
  }

  void initializeShortestPaths()
  {
    city.initializeDijkstraShortestPaths();
    cout << "Shortest paths initialized successfully\n";
  }

  void initializeVehicles()
  {
    chooseOrigin = discrete_distribution<int>(originWeights.begin(), originWeights.end());
    chooseDestination=discrete_distribution<int>(destinationWeights.begin(),destinationWeights.end());
    impatienceTresh=normal_distribution<float>(config.MEAN_IMPATIENCE_THRESHOLD,config.PATIENCE_STDDEV);
    patienceRegen=normal_distribution<float>(config.MEAN_PATIENCE_REGENERATION,0.1f);
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
    cout << "Vehicles initalized\n";
  }

  void reinitializeVehicle(int id) // and calculating trip statistics
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

  void oneStep(int step) 
  {
    stepMovementStats mstats(config);
    vector<int> pendingReinitializations;

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
           if(ftime/tstep + config.correction >= 0.9f)v.currentImpatience =max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction
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
        if(ctime >= v.initTime+v.expectedExternalTime && ctime + config.correction >= nextAllowedEntry[v.currentIntersectionId] && tlmanager.isGreenFor(i,externalRoadId) && (!trafficQueues[currentRoad.id].size() || vehicles[trafficQueues[currentRoad.id].back()].positionOnRoad * currentRoad.lg + config.correction >= config.CAR_LENGTH+config.SAFETY_GAP))
                {
                 canProcessNextVehicle=1;   
                v.actualSpawnTime=nextAllowedEntry[v.currentIntersectionId];
                float ftime=ctime-nextAllowedEntry[v.currentIntersectionId];
                 
                transferToRoad(v,ftime, currentRoad.id,0,i);

                float moveTime=v.positionOnRoad*currentRoad.lg/currentRoad.maxspeed; //on internal Road
                timeSpentMoving=nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime;
                
                 lastMovementTime=1.0f;
              
                 if((nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep+config.correction >= 0.9f)v.currentImpatience =max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction   

                 nextAllowedEntry[v.currentIntersectionId]+= (config.CAR_LENGTH + config.SAFETY_GAP) / currentIntersection.externalSpeed;  
                 
                 tlmanager.recordExtRoadChange(v,currentRoad,step);
                }  
        else if(ctime >= v.initTime+v.expectedExternalTime && ctime + config.correction >= nextAllowedEntry[v.currentIntersectionId])
        {
                
              if(tlmanager.isGreenFor(i,externalRoadId))
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
              float firstOffset = max(0.0f,nextAllowedEntry[i]-ctime) * city.getIntersection(i).externalSpeed ; //changed 

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
            v.currentImpatience =max(0.0f, v.currentImpatience - config.fullStepImpatienceReduction); // impatience_reduction
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

   cout << step << '\n';    
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
  
  void initializeWeights(const string& fileName)
  {

        ifstream file(fileName);
        if(!file.is_open()) 
          throw runtime_error("Cannot open weights data: " + fileName);
        
        bool okDest, okOrig;
        okDest=okOrig=0;
        string csvLine, id, destination_weight, origin_weight;

        getline(file, csvLine); // antet: x,y,id

        int currentId=0;
        while (getline(file, csvLine)) {
            stringstream stream(csvLine);

            getline(stream, id, ',');
            getline(stream, destination_weight, ',');
            getline(stream, origin_weight, ',');

            if(stoi(id) != currentId) 
              throw invalid_argument("Id not in order: " + csvLine);

            if(stoi(destination_weight) < 0)
              throw invalid_argument("Negative destination weight: " + csvLine);

            if(stoi(origin_weight) < 0)
              throw invalid_argument("Negative origin weight: " + csvLine);

            if(stoi(origin_weight) > 0) okOrig=1;
            if(stoi(destination_weight) >0) okDest=1;
            
            originWeights.push_back(stoi(origin_weight));
            destinationWeights.push_back(stoi(destination_weight));
            currentId++;
        }
        
      if(currentId != city.getNoIntersections())
        throw invalid_argument("Weights count does not match no of intersections");

      if(!okOrig)
        throw invalid_argument("At least one origin weight must be non zero");

      if(!okDest)
        throw invalid_argument("At least one destination weight must be non zero");
    } 
  
  void showShortestPaths()
  {
    
    for(int i=0; i< city.getNoIntersections(); i++)
    {
        for(int j=0; j < city.getNoIntersections(); j++)
          cout << city.getNextRoadBetween(i,j) <<  " ";

     cout << '\n';

    }

  }

};

int main() {

  simulationConfig config;
   
  if(!validateAndReportConfig(config))
    return EXIT_FAILURE;
    
  Simulation sim(config);
   try
   {
    sim.readCity();
   }
   catch(const exception& error)
   {
    cerr << "City reading error: " << error.what() << '\n';
    return EXIT_FAILURE;
   }

   sim.setTime(0.0f,0.1f); // tstep must be lower than the time it takes for a vehicle to travel the length of the shortest road at its maximum speed
   sim.initializeShortestPaths();
   
   try {sim.initializeWeights(string(PROJECT_PATH) + "/data/demand.csv");}
   catch(const exception& error)
   {
    cerr << "Weight data reading error: " << error.what() << '\n';
    return EXIT_FAILURE;
   }

   sim.configureTrafficLights();
   sim.initializeVehicles();

    if (!g.is_open()) {
    cerr << "Can't open output.out\n";
    return 1;
    }
   g << config.steps << '\n';
   for(int i=0; i<config.steps; i++)
    {
        sim.oneStep(i);

    }
    return 0;
}
