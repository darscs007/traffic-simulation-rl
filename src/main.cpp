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

#define CAR_LENGTH 1
#define SAFETY_GAP 1.5f
#define INF 1e9
#define steps 4000
#define maxcars 400
#define correction 1e-6
#define defaultGreenSteps 20
#define minGreenSteps 10
#define externalRoadId -2
#define detectionRadius 28
#define isAdaptive 1
#define seed 100
#define IMPATIENCE_PROPAGATION 0.4f // may be a feature in the future
#define MEAN_IMPATIENCE_THRESHOLD 50
#define MEAN_PATIENCE_REGENERATION 0.5f
#define PATIENCE_STDDEV 12
#define IMPATIENCE_MULTIPLIER 1.1f
#define vehicleDetectionRadius 20
#define fullStepImpatienceReduction 0.5f

using namespace std;

string outputFileName = string(PROJECT_PATH) + "/output/output.out";
ofstream g(outputFileName);


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
 int x,y;
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
    float spawnTime; // Time the vehicle spawns -> to be changed !!!
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

class Graph
{
private:
    vector<node> nodes;
    vector<line> lines;
    unordered_map<int, int> nodeIndexMap; // Map to store node id to index mapping
    unordered_map<int, int> lineIndexMap; // Map to store line id to index mapping
    vector<vector<int>> adjList; // Adjacency list to store the graph structure
    vector<vector<float>> timeMatrix;
public:
    line getLine(int id)
    {
        return lines[id];

    }

    node getIntersection(int id)
    {
      return nodes[id];

    }

    void addNode(int x, int y, int externalSpeed,int id)
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
        string csvLine, x, y, externalSpeed,id;

        getline(file, csvLine); // antet: x,y,id

        while (getline(file, csvLine)) {
            stringstream stream(csvLine);

            getline(stream, x, ',');
            getline(stream, y, ',');
            getline(stream,externalSpeed,',');
            getline(stream, id, ',');
            int idi=stoi(id);
           
            if (nodeIndexMap.find(idi) != nodeIndexMap.end()) {
                cout << "Duplicate intersection id found: " << idi << "; Terminating progam.\n";
                exit(1);
            }

            addNode(stoi(x), stoi(y), stoi(externalSpeed),stoi(id));
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

            int ok=1;

            if(nodeIndexMap.find(fromi) == nodeIndexMap.end()) {
                cout << "Invalid 'from' intersection id: " << fromi << "; ";
                ok=0;
               
            }

            if(nodeIndexMap.find(toi) == nodeIndexMap.end()) {
                cout << "Invalid 'to' intersection id: " << toi << "; ";
                ok=0;   
                
            }

            if(fromi == toi) {
                cout << "Road cannot connect the same intersection: " << fromi << "; ";
                ok=0;
               
            }

            if(lgi <= 0) {
                cout << "Invalid road length: " << lgi << "; ";
                ok=0;
                
            }

            if(maxspeedi <= 0) {
                cout << "Invalid road max speed: " << maxspeedi << "; ";
                ok=0;
                
            }

            if(idi < 0) {
                cout << "Invalid road id: " << idi << "; ";
                ok=0;
                
            }

            if(lineIndexMap.find(idi) != lineIndexMap.end()) {
                cout << "Duplicate road id found: " << idi << "; ";
                ok=0;
                
            }

            if(!ok) {
                cout << "Terminating program due to invalid road entry: " << csvLine << '\n';
                exit(1);
            }

            

            addLine(fromi, toi, lgi, maxspeedi, idi);
        }
    }

   vector<vector<int>> initializeDijkstraShortestPaths()
   {
    
    vector<vector<int>> shortestPaths(nodes.size(), vector<int>(nodes.size(), -1)), successors(nodes.size(),vector<int>(nodes.size(), -1));

   //dijkstra O(n^2) for each node, total O(n^3). I will optimize it later with a priority queue O(n^2 log n)
    for(int i=0;i<nodes.size();i++)
    {

        vector<int> f(nodes.size(), 0);
        vector<float> d(nodes.size(), INF);

      
        d[i]=0;
        f[i]=1;

       for(int j=0; j<adjList[i].size(); j++)
            {
            line cline=lines[adjList[i][j]];
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
            line cline=lines[adjList[x][j]];
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
               line currentRoad= getLine(shortestPaths[i][dest]);
               toAdd+=(float) currentRoad.lg/ currentRoad.maxspeed;

               timeMatrix[currentRoad.from][j]=toAdd;
               dest=currentRoad.from;
             }
             
            }
    
    int x;
    
    for(int i=0; i<nodes.size();i++)
        for(int j=0; j<nodes.size(); j++)
               getShortestPath(shortestPaths,successors,i,j,x);

    return successors;
   }
    
   int getNoIntersections()
   {
    return nodes.size();
   }

   int getNoRoads()
   {
    return lines.size();
   }

   vector<int> getAdjRoads(int id) // number of roads that start in the intersection with this id
   {
    return adjList[id];
   }

   void showTimeMatrix()
   {
    for(int i=0; i<timeMatrix.size(); i++)
        {for(int j=0; j<timeMatrix[i].size(); j++) cout << timeMatrix[i][j] << " ";
         cout << '\n';
        }
   }

   float getTimeBetween(int i, int j)
   {
    return timeMatrix[i][j];
   }
};

class Simulation
{
 private:
  Graph city;
  vector<vehicle> vehicles;
  float ctime; // current time of the simulation
  float tstep; // time step for the simulation
  vector<vector<int>> shortPaths; // 2D vector to store shortest paths between intersections
  vector<int> originWeights,destinationWeights;
  discrete_distribution<int> chooseOrigin,chooseDestination;
  mt19937 rng; // generate random number from seed
  mt19937 patienceRng; 
  vector<queue<int>> waitQueues;
  vector<deque<int>> trafficQueues;
  vector<float> nextAllowedEntry; // vector to store the next allowed entry time for each intersection
  vector<trafficLight> trafficLights; // vector to store traffic light configurations for each intersection
  vector<int> incomingPhaseId;
  vector<int> outgoingPhaseId;
  normal_distribution<float> impatienceTresh;
  normal_distribution<float> patienceRegen;

  float taExtTime=0.0f, teExtTime=0.0f, taIntTime=0.0f, teIntTime=0.0f, sumExtDiv=0.0f, sumIntDiv=0.0f, sumTotDiv=0.0f;
  int noTrips=0;

 public:
  Simulation(int seedValue)
    :rng(seedValue), patienceRng(seedValue)
    {}

  void configureTrafficLights()
  {
    trafficLights.resize(city.getNoIntersections());
    
    if(isAdaptive)
    {
    incomingPhaseId.assign(city.getNoRoads(), -1);
    outgoingPhaseId.assign(city.getNoRoads(), -1);
    }
    
    for (int i = 0; i < city.getNoIntersections(); i++)
    {
        trafficLights[i].id = i;
        trafficLights[i].roadsTL.push_back({externalRoadId,defaultGreenSteps,0}); // adding the external road
        trafficLights[i].currentState = 0;
        trafficLights[i].nextChangeStep = trafficLights[i].roadsTL[0].greenSteps;
    }
    
    for(int i=0; i<city.getNoIntersections(); i++)
    {
        vector<int> adjRoads=city.getAdjRoads(i);
        int ind=1;
        for(int j=0; j<adjRoads.size(); j++)
        {
            line currentRoad=city.getLine(adjRoads[j]);
            trafficLights[currentRoad.to].roadsTL.push_back({currentRoad.id,defaultGreenSteps});
            if(isAdaptive)
            {
            incomingPhaseId[currentRoad.id]=trafficLights[currentRoad.to].roadsTL.size()-1;
            outgoingPhaseId[currentRoad.id]=ind++; //column number, 0 is the external road, so we start from 1
            }
        }
    }

    if(isAdaptive)
        for(int i=0; i<trafficLights.size(); i++)
            trafficLights[i].roadChangeMatrix.resize(trafficLights[i].roadsTL.size(), vector<int>(city.getAdjRoads(i).size()+1,0));
       
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
    shortPaths = city.initializeDijkstraShortestPaths();
    city.showTimeMatrix();
    cout << "Shortest paths initialized successfully\n";
  }

  void initializeVehicles()
  {
    chooseOrigin = discrete_distribution<int>(originWeights.begin(), originWeights.end());
    chooseDestination=discrete_distribution<int>(destinationWeights.begin(),destinationWeights.end());
    impatienceTresh=normal_distribution<float>(MEAN_IMPATIENCE_THRESHOLD,PATIENCE_STDDEV);
    patienceRegen=normal_distribution<float>(MEAN_PATIENCE_REGENERATION,0.1f);
    waitQueues.resize(city.getNoIntersections());
    trafficQueues.resize(city.getNoRoads());
    vehicles.resize(maxcars);
    nextAllowedEntry.resize(city.getNoIntersections(), 0.0f);

    for(int i=0; i<maxcars; i++)
    {
      vehicle v{};
      v.lastStepProcessed=-1;
      v.destinationIntersectionId=chooseDestination(rng);
      v.currentIntersectionId=chooseOrigin(rng);
      while(v.currentIntersectionId==v.destinationIntersectionId) v.currentIntersectionId=chooseOrigin(rng);
      v.startIntersectionId=v.currentIntersectionId;
      waitQueues[v.currentIntersectionId].push(i);

      node currentIntersection=city.getIntersection(v.currentIntersectionId);

      v.id=i;
      v.spawnTime=v.expectedExternalTime=(waitQueues[v.currentIntersectionId].size()-1) * (CAR_LENGTH + SAFETY_GAP) / currentIntersection.externalSpeed;
      v.initTime=0.0f;
      
      v.impatienceThreshold=impatienceTresh(patienceRng);
      while(v.impatienceThreshold < MEAN_IMPATIENCE_THRESHOLD-30 || v.impatienceThreshold > MEAN_IMPATIENCE_THRESHOLD+30) v.impatienceThreshold=impatienceTresh(patienceRng);
      v.patienceRegen=patienceRegen(patienceRng);
      while(v.patienceRegen < 0.0f || v.patienceRegen > 1.0f) v.patienceRegen=patienceRegen(patienceRng);
      v.currentImpatience=0.0f;
      v.isImpatient=0;

      vehicles[i] = v;
    }
    cout << "Vehicles initalized\n";
  }

  void reinitializeVehicle(int id)
  {
     vehicle &v=vehicles[id];
    
    if(v.spawnTime)
     {
      noTrips++;
    
      teIntTime+=city.getTimeBetween(v.startIntersectionId,v.destinationIntersectionId);
      taIntTime+=v.endTime-v.actualSpawnTime;
      sumIntDiv+= (v.endTime-v.actualSpawnTime)/city.getTimeBetween(v.startIntersectionId,v.destinationIntersectionId);
      
      sumTotDiv+= (v.endTime-v.initTime)/(v.expectedExternalTime+city.getTimeBetween(v.startIntersectionId,v.destinationIntersectionId));

      sumExtDiv+= (v.actualSpawnTime-v.initTime)/v.expectedExternalTime;
      teExtTime+=v.expectedExternalTime; //total expected=  te, total actual = ta
      taExtTime+=v.actualSpawnTime-v.initTime;
      }
      
      v.destinationIntersectionId=chooseDestination(rng);
      v.currentIntersectionId=chooseOrigin(rng);
      while(v.currentIntersectionId==v.destinationIntersectionId) v.currentIntersectionId=chooseOrigin(rng);
      v.startIntersectionId=v.currentIntersectionId;
      v.positionOnRoad=0;
      if(!waitQueues[v.currentIntersectionId].empty()) 
      {
        if(ctime+tstep > vehicles[waitQueues[v.currentIntersectionId].back()].spawnTime + (CAR_LENGTH + SAFETY_GAP) / city.getIntersection(v.currentIntersectionId).externalSpeed)v.spawnTime=ctime+tstep;
        else {v.spawnTime=vehicles[waitQueues[v.currentIntersectionId].back()].spawnTime + (CAR_LENGTH + SAFETY_GAP) / city.getIntersection(v.currentIntersectionId).externalSpeed; }
        v.expectedExternalTime=(waitQueues[v.currentIntersectionId].size()+1) *(CAR_LENGTH + SAFETY_GAP)/city.getIntersection(v.currentIntersectionId).externalSpeed ;
    }
      else {v.spawnTime=ctime+tstep; v.expectedExternalTime=tstep;}
      
      if(!waitQueues[v.currentIntersectionId].size()) nextAllowedEntry[v.currentIntersectionId]=v.spawnTime;
      
      v.lastStepProcessed=-1;
      v.isActive=0;
      v.initTime=ctime;
      v.endTime=0;
      v.actualSpawnTime=0;
      v.currentImpatience = 0.0f;
      v.forbiddenRoads.clear();
      v.isImpatient=0;

      waitQueues[v.currentIntersectionId].push(v.id);

  }

  void updateTrafficLights(int step)
  {

    for(int i=0; i<trafficLights.size(); i++)
    {
        if( isAdaptive && step &&step % (trafficLights[i].roadsTL.size()*defaultGreenSteps) == 0)
        {
         float maxscore=0.0f, distance=0.0f;
         while(distance <= detectionRadius) 
         {
           maxscore+=1-distance/detectionRadius;
           distance+=CAR_LENGTH+SAFETY_GAP; 
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

         if(tscore-correction>=0)
         { 
         int totalSteps=0;
         int usedSteps=0;

         for(int j=0; j<trafficLights[i].roadsTL.size(); j++)
            {
             
             trafficLights[i].roadsTL[j].greenSteps=  minGreenSteps;
             usedSteps+=trafficLights[i].roadsTL[j].greenSteps;
             totalSteps+=trafficLights[i].roadsTL[j].greenSteps;
            }
         
         for(int j=0; j<trafficLights[i].roadsTL.size(); j++)
            {
             int extraSteps= trafficLights[i].roadsTL[j].score*trafficLights[i].roadsTL[j].availability/ tscore * (trafficLights[i].roadsTL.size() * defaultGreenSteps - usedSteps);
             trafficLights[i].roadsTL[j].greenSteps+= extraSteps ;
             totalSteps+=extraSteps;
            }
        
         totalSteps=trafficLights[i].roadsTL.size()*defaultGreenSteps - totalSteps;
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
             trafficLights[i].roadsTL[j].greenSteps=defaultGreenSteps;
             
         }


        }
        
        if(step >= trafficLights[i].nextChangeStep)
        {
           trafficLights[i].currentState = (trafficLights[i].currentState+1) % trafficLights[i].roadsTL.size();
           trafficLights[i].nextChangeStep += trafficLights[i].roadsTL[trafficLights[i].currentState].greenSteps;
        }
           
        

    }
    

    if(isAdaptive && step % defaultGreenSteps==0) 
      for(int i=0; i<trafficLights.size(); i++)
        for (int e = 0; e < trafficLights[i].roadsTL.size(); e++)
            trafficLights[i].roadsTL[e].score= 0.0f;
  }

  void oneStep(int step) 
  {
    int MovingVehiclesNo, StationaryVehiclesNo;
    float fMovingNo, fStationaryNo;
    MovingVehiclesNo=StationaryVehiclesNo=0;
    fMovingNo=fStationaryNo=0.0f;
    vector<int> pendingReinitializations;

    updateTrafficLights(step);

    for(int i=0; i<city.getNoRoads(); i++)
      {
        float nextAdvance=-1;
        int status=0; // 0 first vehicle, unitialized, 1 moved freely without reaching the end, 2 reached the end while moving or has an obstacle ahead, could not exit , 3 exited the road
        
        for(int j=0; j<trafficQueues[i].size(); j++)
        {
        bool changedPaths=0;    
        line currentRoad=city.getLine(i);
        vehicle &v=vehicles[trafficQueues[i][j]];

         if(v.lastStepProcessed==step) continue;
         

         line initRoad=currentRoad;
         float initPos=v.positionOnRoad; 
         bool didNotExit=1;
         
         v.lastStepProcessed=step;
         if(status==0 || status>=2)
         {
         
          nextAdvance=v.positionOnRoad;

          if(status == 0 || status == 3)  v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
          else  
            if(tstep * currentRoad.maxspeed / currentRoad.lg < vehicles[trafficQueues[i][j-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg-v.positionOnRoad )  
               {
                v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
                status=1; 
                
                }
            else 
               {
                float ftime= tstep - (vehicles[trafficQueues[i][j-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg-v.positionOnRoad)*currentRoad.lg/currentRoad.maxspeed;
                
                v.positionOnRoad = vehicles[trafficQueues[i][j-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg; 
                status=2;
   
                trafficLight tl=trafficLights[currentRoad.to]; // patience
                if(tl.roadsTL[tl.currentState].roadId == currentRoad.id && (1-v.positionOnRoad)*currentRoad.lg <= vehicleDetectionRadius) 
                {
                 ftime = clamp(ftime, 0.0f, tstep); 
                 v.currentImpatience+=ftime/tstep;
                 v.currentImpatience*=IMPATIENCE_MULTIPLIER;
                    
                }

               }

         nextAdvance=v.positionOnRoad-nextAdvance;
            
         if(v.positionOnRoad >=1)
            {
             trafficLight tl=trafficLights[currentRoad.to];
             
             if(currentRoad.to == v.destinationIntersectionId) 
              {
                   if(tl.roadsTL[tl.currentState].roadId == currentRoad.id) 
                    {
                    MovingVehiclesNo++;
                    float ftime=(1-initPos) * currentRoad.lg/currentRoad.maxspeed;
                    fMovingNo+=ftime/tstep;
                    v.endTime=ctime-(tstep-ftime);

                    didNotExit=0;
                    status=3; // out of road
                    pendingReinitializations.push_back(trafficQueues[i][j]);
                    trafficQueues[i].pop_front();
                    j--;

                    if(isAdaptive)trafficLights[v.destinationIntersectionId].roadChangeMatrix[incomingPhaseId[v.currentRoadId]][0]++;
                    }
                   else 
                   {
                    status=2;
                    v.positionOnRoad=1;
                   }
              } 
             else 
               {
                float ftime= (v.positionOnRoad - 1) * currentRoad.lg / currentRoad.maxspeed;
                int ok=0;
                status=2;//attempts to reach the next road
                if(tl.roadsTL[tl.currentState].roadId == currentRoad.id)
                {
                ok++;
                
                int initialRoadId = currentRoad.id;
                currentRoad= city.getLine(shortPaths[currentRoad.to][v.destinationIntersectionId]);
                if(!trafficQueues[currentRoad.id].size() || vehicles[trafficQueues[currentRoad.id].back()].positionOnRoad * currentRoad.lg >= CAR_LENGTH+SAFETY_GAP)
                {
                    
                    v.currentRoadId=currentRoad.id;
                    v.currentIntersectionId=currentRoad.from;
                    
                    if(trafficQueues[currentRoad.id].size() && vehicles[trafficQueues[currentRoad.id][trafficQueues[currentRoad.id].size()-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg <ftime * currentRoad.maxspeed / currentRoad.lg) 
                    {
                     v.positionOnRoad =  vehicles[trafficQueues[currentRoad.id][trafficQueues[currentRoad.id].size()-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg;   
                    }
                    else  
                        v.positionOnRoad=ftime * currentRoad.maxspeed / currentRoad.lg;
                    
                    trafficQueues[currentRoad.id].push_back(v.id);
                    trafficQueues[i].pop_front();
                    status=3; // reached the next road and exited the current road
                    j--; ok++;

                    if(isAdaptive)
                    {
                       trafficLights[city.getLine(initialRoadId).to].roadChangeMatrix[incomingPhaseId[initialRoadId]][outgoingPhaseId[v.currentRoadId]]++; 
                     
                       if(v.currentRoadId < i && (step+1) % defaultGreenSteps == 0)
                            trafficLights[currentRoad.to].roadsTL[incomingPhaseId[currentRoad.id]].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ detectionRadius);    
                    }
                    
                 }
                 else //impatience processing and rerouting
                 {
                  v.currentImpatience+=ftime/tstep;
                  v.currentImpatience*=IMPATIENCE_MULTIPLIER;
                  
                  if(v.currentImpatience > v.impatienceThreshold)
                  {
                    v.isImpatient=1;
                    int nextBestRoadId=-1;
                    float nextBestTime=INF;

                    vector<int> adjRoads=city.getAdjRoads(city.getLine(v.currentRoadId).to);
                    for(int i=0; i< adjRoads.size(); i++)
                    {
                     line possibleRoad=city.getLine(adjRoads[i]);
                     if(currentRoad.id == adjRoads[i]) continue; //current road is, here, the next road in the initial path, it has been initialized before if
                     if(possibleRoad.to != v.destinationIntersectionId && city.getLine(shortPaths[possibleRoad.to][v.destinationIntersectionId]).to == initRoad.to) continue;
                     bool isNotForbidden=1;
                     for(int j=0; j<v.forbiddenRoads.size() && isNotForbidden; j++)
                       if(v.forbiddenRoads[j] == possibleRoad.id) isNotForbidden=0;

                     if(!isNotForbidden) continue;

                     if(trafficQueues[possibleRoad.id].size() && vehicles[trafficQueues[possibleRoad.id].back()].positionOnRoad * possibleRoad.lg < CAR_LENGTH+SAFETY_GAP) continue;

                     if((float)possibleRoad.lg/possibleRoad.maxspeed + city.getTimeBetween(possibleRoad.to, v.destinationIntersectionId) < nextBestTime)
                     {
                      nextBestRoadId=possibleRoad.id;
                      nextBestTime=(float)possibleRoad.lg/possibleRoad.maxspeed + city.getTimeBetween(possibleRoad.to, v.destinationIntersectionId);
                     }
                    }
                  
                    if(nextBestRoadId != -1)
                    {
                     v.forbiddenRoads.push_back(nextBestRoadId);
                     v.currentRoadId=nextBestRoadId;
                     v.currentIntersectionId=city.getLine(nextBestRoadId).from;   
                     currentRoad=city.getLine(nextBestRoadId);
                     if(trafficQueues[currentRoad.id].size() && vehicles[trafficQueues[currentRoad.id][trafficQueues[currentRoad.id].size()-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg <ftime * currentRoad.maxspeed / currentRoad.lg) 
                    {
                     v.positionOnRoad =  vehicles[trafficQueues[currentRoad.id][trafficQueues[currentRoad.id].size()-1]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg;   
                    }
                    else  
                        v.positionOnRoad=ftime * currentRoad.maxspeed / currentRoad.lg;
                    
                    v.currentImpatience-=v.patienceRegen*v.currentImpatience;
                    trafficQueues[currentRoad.id].push_back(v.id);
                    trafficQueues[i].pop_front();
                    status=3; // reached the next road and exited the current road
                    j--; ok++;   
                    changedPaths=1;

                    if(isAdaptive)
                    {
                       trafficLights[city.getLine(initialRoadId).to].roadChangeMatrix[incomingPhaseId[initialRoadId]][outgoingPhaseId[v.currentRoadId]]++; 
                     
                       if(v.currentRoadId < i && (step+1) % defaultGreenSteps == 0)
                            trafficLights[currentRoad.to].roadsTL[incomingPhaseId[currentRoad.id]].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ detectionRadius);    
                    }
                    }
                   }
                   

                 }
               }
               
               if(ok<2) v.positionOnRoad=1; 

               }
            }

           }
           else v.positionOnRoad+=nextAdvance; // moving freely, but not reaching the enf of the road (status == 1)
           
         
         if(didNotExit){
            float fMovingInitial=fMovingNo;
            line finalRoad=city.getLine(v.currentRoadId);
           if(initRoad.id != finalRoad.id)
           {
            MovingVehiclesNo++;
            float ftime= (1-initPos) * initRoad.lg/initRoad.maxspeed + v.positionOnRoad * finalRoad.lg/finalRoad.maxspeed;
            fMovingNo+=ftime/tstep;
            fStationaryNo+=1-ftime/tstep;
            
           }
           else
           {
            float ftime=(v.positionOnRoad-initPos) * initRoad.lg/initRoad.maxspeed;
            if(ftime>correction) 
            {
             MovingVehiclesNo++; 
             if(ftime + correction < tstep)
             {
             fMovingNo+=ftime/tstep; 
             fStationaryNo+=1-ftime/tstep;
             }
             else 
             {
                fMovingNo++;
                
             }
            }
            else {StationaryVehiclesNo++; fStationaryNo++;}
            
           }

           if(fMovingNo-fMovingInitial + correction >= 0.9f)v.currentImpatience =max(0.0f, v.currentImpatience - fullStepImpatienceReduction); // impatience_reduction
           }   
          v.isImpatient = changedPaths || v.currentImpatience > v.impatienceThreshold;
         }
        
         line currentRoad=city.getLine(i);
         if(isAdaptive && (step+1) % defaultGreenSteps == 0)
          {  
            int ind;  
            for(int e=0; e<trafficLights[currentRoad.to].roadsTL.size(); e++)
                if(trafficLights[currentRoad.to].roadsTL[e].roadId==currentRoad.id) {ind=e;break;}
            for(int j=0; j<trafficQueues[i].size(); j++)
            {
             
             vehicle v=vehicles[trafficQueues[i][j]];   
             trafficLights[currentRoad.to].roadsTL[ind].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ detectionRadius);  
            }
          } 
      }
     

      for(int i=0; i<waitQueues.size(); i++)
        {
         bool ok=1; int NoInitVehicles=waitQueues[i].size();
         
         float lastMovementTime=0; // the last initialized vehicle movement time; 
         float iNextEntry=nextAllowedEntry[i];
         while(ok && !waitQueues[i].empty())
         {
         bool changedPaths=0;
         ok=0;
         vehicle &v=vehicles[waitQueues[i].front()];  
         v.currentRoadId = shortPaths[v.currentIntersectionId][v.destinationIntersectionId]; 
         line currentRoad = city.getLine(v.currentRoadId);
         node currentIntersection = city.getIntersection(v.currentIntersectionId);
         trafficLight tl=trafficLights[currentIntersection.id];  

        //if the vehicle has not yet left the waiting queue
        NoInitVehicles--;
        if(nextAllowedEntry[v.currentIntersectionId] < v.spawnTime) 
        {
         nextAllowedEntry[v.currentIntersectionId] = v.spawnTime;

        }
        if(ctime >= v.spawnTime && ctime + correction >= nextAllowedEntry[v.currentIntersectionId] && tl.roadsTL[tl.currentState].roadId == externalRoadId && (!trafficQueues[currentRoad.id].size() || vehicles[trafficQueues[currentRoad.id].back()].positionOnRoad * currentRoad.lg + correction >= CAR_LENGTH+SAFETY_GAP))
                {
                 ok=1;   
                 v.isActive=1;
                 
                v.actualSpawnTime=nextAllowedEntry[v.currentIntersectionId];
                float ftime=ctime-nextAllowedEntry[v.currentIntersectionId];

                 trafficQueues[currentRoad.id].push_back(v.id);
                 if(trafficQueues[currentRoad.id].size() == 1) {v.positionOnRoad=ftime * currentRoad.maxspeed / currentRoad.lg; }
                 else v.positionOnRoad = min(vehicles[trafficQueues[currentRoad.id][trafficQueues[currentRoad.id].size()-2]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg, ftime * currentRoad.maxspeed / currentRoad.lg);

                 float moveTime=v.positionOnRoad*currentRoad.lg/currentRoad.maxspeed;
                 MovingVehiclesNo++;
                 fMovingNo+=(nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep;
                 fStationaryNo+= 1- (nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep;
                 lastMovementTime=1;
                 if((nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep+correction >=1)v.currentImpatience =max(0.0f, v.currentImpatience - fullStepImpatienceReduction); // impatience_reduction   

                 waitQueues[v.currentIntersectionId].pop();  
                 nextAllowedEntry[v.currentIntersectionId]+= (CAR_LENGTH + SAFETY_GAP) / currentIntersection.externalSpeed;  
                 
                 if(isAdaptive)
                    {
                    trafficLights[v.currentIntersectionId].roadChangeMatrix[0][outgoingPhaseId[currentRoad.id]]++;

                    if((step+1) % defaultGreenSteps == 0)
                        trafficLights[currentRoad.to].roadsTL[incomingPhaseId[currentRoad.id]].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ detectionRadius);
                    }
                }  
        else if(ctime >= v.spawnTime && ctime + correction >= nextAllowedEntry[v.currentIntersectionId])
        {
              bool exited=0;  
              if(tl.roadsTL[tl.currentState].roadId == externalRoadId)
              {
                v.currentImpatience+=1-(nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep; lastMovementTime=(nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep;
                v.currentImpatience*=IMPATIENCE_MULTIPLIER;

                if(v.currentImpatience > v.impatienceThreshold)
                  {
                    v.isImpatient=1;
                    int nextBestRoadId=-1;
                    float nextBestTime=INF;

                    vector<int> adjRoads=city.getAdjRoads(i);
                    for(int i=0; i< adjRoads.size(); i++)
                    {
                     line possibleRoad=city.getLine(adjRoads[i]);
                     if(v.currentRoadId == adjRoads[i]) continue;
                     if(possibleRoad.to != v.destinationIntersectionId && city.getLine(shortPaths[possibleRoad.to][v.destinationIntersectionId]).to == v.currentIntersectionId) continue;
                     bool isNotForbidden=1;
                     for(int j=0; j<v.forbiddenRoads.size() && isNotForbidden; j++)
                       if(v.forbiddenRoads[j] == possibleRoad.id) isNotForbidden=0;

                     if(!isNotForbidden) continue;

                     if(trafficQueues[possibleRoad.id].size() && vehicles[trafficQueues[possibleRoad.id].back()].positionOnRoad * possibleRoad.lg < CAR_LENGTH+SAFETY_GAP) continue;

                     if((float)possibleRoad.lg/possibleRoad.maxspeed + city.getTimeBetween(possibleRoad.to, v.destinationIntersectionId) < nextBestTime)
                     {
                      nextBestRoadId=possibleRoad.id;
                      nextBestTime=(float)possibleRoad.lg/possibleRoad.maxspeed + city.getTimeBetween(possibleRoad.to, v.destinationIntersectionId);
                     }
                    }
                  
                if(nextBestRoadId != -1)
                    {
                 
                 exited=1;
                 ok=1;   
                 v.isActive=1;
                 v.currentRoadId=nextBestRoadId;
                 v.forbiddenRoads.push_back(nextBestRoadId);
                 v.currentImpatience -= v.patienceRegen * v.currentImpatience;
                 currentRoad=city.getLine(nextBestRoadId);
                v.actualSpawnTime=nextAllowedEntry[v.currentIntersectionId];
                float ftime=ctime-nextAllowedEntry[v.currentIntersectionId];

                 trafficQueues[currentRoad.id].push_back(v.id);
                 if(trafficQueues[currentRoad.id].size() == 1) {v.positionOnRoad=ftime * currentRoad.maxspeed / currentRoad.lg; }
                 else v.positionOnRoad = min(vehicles[trafficQueues[currentRoad.id][trafficQueues[currentRoad.id].size()-2]].positionOnRoad-(CAR_LENGTH+SAFETY_GAP)/currentRoad.lg, ftime * currentRoad.maxspeed / currentRoad.lg);

                 float moveTime=v.positionOnRoad*currentRoad.lg/currentRoad.maxspeed;
                 MovingVehiclesNo++;
                 fMovingNo+=(nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep;
                 fStationaryNo+= 1- (nextAllowedEntry[v.currentIntersectionId]-iNextEntry+moveTime)/tstep;
                 lastMovementTime=1;

                 waitQueues[v.currentIntersectionId].pop();  
                 nextAllowedEntry[v.currentIntersectionId]+= (CAR_LENGTH + SAFETY_GAP) / currentIntersection.externalSpeed;  
                 changedPaths=1;
                 if(isAdaptive)
                    {
                    trafficLights[v.currentIntersectionId].roadChangeMatrix[0][outgoingPhaseId[currentRoad.id]]++;

                    if((step+1) % defaultGreenSteps == 0)
                        trafficLights[currentRoad.to].roadsTL[incomingPhaseId[currentRoad.id]].score += max(0.0f, 1.0f - (1.0f - v.positionOnRoad) *currentRoad.lg/ detectionRadius);
                    }
                    }
                   }

              }

          if(!exited)
            {
            if(nextAllowedEntry[v.currentIntersectionId] - iNextEntry < correction) 
              {StationaryVehiclesNo++; fStationaryNo++; lastMovementTime=0;}
              else
              {
                MovingVehiclesNo++; 
                fMovingNo+= (nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep; 
                fStationaryNo+= 1-(nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep; 
                lastMovementTime=(nextAllowedEntry[v.currentIntersectionId] - iNextEntry)/tstep;
                
             }  
                
              nextAllowedEntry[v.currentIntersectionId]=ctime;   
             }
         }
        else
        {
            MovingVehiclesNo++;
            fMovingNo++;
            v.currentImpatience =max(0.0f, v.currentImpatience - fullStepImpatienceReduction); // impatience_reduction
            lastMovementTime=1;
        }
        
    
        v.isImpatient =changedPaths || v.currentImpatience > v.impatienceThreshold;
        } 
        if(!lastMovementTime) {StationaryVehiclesNo+=NoInitVehicles; fStationaryNo+=NoInitVehicles;}
        else{MovingVehiclesNo+=NoInitVehicles; fMovingNo+=NoInitVehicles * lastMovementTime; fStationaryNo+= NoInitVehicles* (1-lastMovementTime);}  
        
        if(isAdaptive && (step+1) % defaultGreenSteps == 0) 
        {
         float firstOffset = max(0.0f,nextAllowedEntry[i]-(ctime + tstep))*city.getIntersection(i).externalSpeed; 
         for(int e=0; e < waitQueues[i].size(); e++)
           if( 1.0f - firstOffset/ detectionRadius >0 )  {trafficLights[i].roadsTL[0].score +=  1.0f - firstOffset/ detectionRadius; firstOffset+=CAR_LENGTH+SAFETY_GAP;}
           else break;
        }

        
           
        } 
         
     
    for(int i=0; i<pendingReinitializations.size(); i++)
       reinitializeVehicle(pendingReinitializations[i]);

   cout << step << '\n';    
    int nr=0;
    
    g << noTrips << " ";
    if(noTrips) g << sumExtDiv/noTrips << " " << sumIntDiv/noTrips << " " << sumTotDiv/noTrips << " " << taExtTime/teExtTime << " " << taIntTime/teIntTime << " " << (taExtTime+taIntTime)/(teExtTime+teIntTime) << '\n'; 
    else g << 0 << '\n';

    if(step)g << MovingVehiclesNo << " " << StationaryVehiclesNo << " " << fMovingNo << " " << fStationaryNo << '\n';
    else g << 0 <<" " <<0<<  " " << 0 << " " << 0 << '\n';

    for(int i=0; i<trafficLights.size(); i++)
        g << trafficLights[i].roadsTL[trafficLights[i].currentState].roadId << " ";
    g << '\n';
    
    for(int i=0; i<vehicles.size(); i++)
        if(vehicles[i].isActive) nr++;
    
    g << nr << '\n';
    
    for(int i=0; i<vehicles.size(); i++)
    if(vehicles[i].isActive)
        {
            vehicle v=vehicles[i];
            line r=city.getLine(v.currentRoadId);
  
            g << v.id << " " << v.currentIntersectionId << " " << r.to << " " << v.currentRoadId << " " << v.positionOnRoad << " " << v.isImpatient<<'\n';
        }
    
    ctime+=tstep;    
    
  }

  void initializeWeights(const string& fileName)
  {
        int n=city.getNoIntersections();
      

        ifstream file(fileName);
        string csvLine, id, destination_weight, origin_weight;

        getline(file, csvLine); // antet: x,y,id

        while (getline(file, csvLine)) {
            stringstream stream(csvLine);

            getline(stream, id, ',');
            getline(stream, destination_weight, ',');
            getline(stream, origin_weight, ',');
           
            originWeights.push_back(stoi(origin_weight));
            destinationWeights.push_back(stoi(destination_weight));
        }

        
    } 
  
  void showShortestPaths()
  {
    
    for(int i=0; i< shortPaths.size(); i++)
    {
        for(int j=0; j < shortPaths[i].size(); j++)
          cout << shortPaths[i][j] <<  " ";

     cout << '\n';

    }

  }

};

int main() {
   
   Simulation sim(seed);
   
   sim.readCity();
   sim.setTime(0.0f,0.1f); // tstep must be lower than the time it takes for a vehicle to travel the length of the shortest road at its maximum speed
   sim.initializeShortestPaths();
   sim.initializeWeights(string(PROJECT_PATH) + "/data/demand.csv");
   sim.configureTrafficLights();
   sim.initializeVehicles();

    if (!g.is_open()) {
    cerr << "Can't open output.out\n";
    return 1;
    }
   g << steps << '\n';
   for(int i=0; i<steps; i++)
    {
        sim.oneStep(i);

    }
    return 0;
}
