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

#define CAR_LENGTH 1
#define SAFETY_GAP 1
#define INF 1e9
#define steps 1000
# define maxcars 100

using namespace std;


struct node //intersection
{
 int x,y;
 int id;
};

struct line // road
{
 int from, to;
 int lg ,maxspeed, id;
 queue<int> vehiclesOnRoad; // Queue to store vehicle IDs on the road
 int maxVehicles; // Maximum number of vehicles allowed on the road
};

struct vehicle
{
    int id;
    int currentIntersectionId;
    int destinationIntersectionId;
    int currentRoadId;
    float positionOnRoad; // Position on the road as a float between 0.0 and 1.0
    float waitTime; // Time the vehicle has been waiting at the intersection
    float spawnTime; // Time the vehicle was spawned
    int lcar; //length of the car
    //speed is the road's maxspeed
    
};

class Graph
{
private:
    vector<node> nodes;
    vector<line> lines;
    unordered_map<int, int> nodeIndexMap; // Map to store node id to index mapping
    unordered_map<int, int> lineIndexMap; // Map to store line id to index mapping
    vector<vector<int>> adjList; // Adjacency list to store the graph structure



public:
    line getLine(int id)
    {
        return lines[id];

    }

    node getIntersection(int id)
    {
      return nodes[id];

    }

    void addNode(int x, int y, int id)
       {
        nodes.push_back({x,y,id});
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
        string csvLine, x, y, id;

        getline(file, csvLine); // antet: x,y,id

        while (getline(file, csvLine)) {
            stringstream stream(csvLine);

            getline(stream, x, ',');
            getline(stream, y, ',');
            getline(stream, id, ',');
            int idi=stoi(id);

            if (nodeIndexMap.find(idi) != nodeIndexMap.end()) {
                cout << "Duplicate intersection id found: " << idi << "; Terminating progam.\n";
                exit(1);
            }

            addNode(stoi(x), stoi(y), stoi(id));
        }
    }


    void getShortestPath(vector<vector<int>> &a, vector<vector<int>> &b, int source, int destination, int &x)//calculates the successors
    {
      if(source == destination || b[source][destination]!=-1) return;
        
        
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
  int seed;
  discrete_distribution<int> chooseOrigin,chooseDestination;  
  mt19937 rng;

 public:
  Simulation(int seedValue)
    :seed(seedValue), rng(seedValue)
    {}
 
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
    cout << "Shortest paths initialized successfully\n";
  }

  void initializeVehicles()
  {
    chooseOrigin = discrete_distribution<int>(originWeights.begin(), originWeights.end());
    chooseDestination=discrete_distribution<int>(destinationWeights.begin(),destinationWeights.end());
    vehicles.resize(maxcars);
    for(int i=0; i<maxcars; i++)
    {
      vehicle v;
      v.destinationIntersectionId=chooseDestination(rng);
      v.currentIntersectionId=chooseOrigin(rng);
      while(v.currentIntersectionId==v.destinationIntersectionId) v.currentIntersectionId=chooseOrigin(rng);
      v.positionOnRoad=0;
      v.id=i;
      v.spawnTime=0.1f*i;
      vehicles[i] = v;

    }

  }

  void reinitializeVehicle(vehicle &v)
  {
      v.destinationIntersectionId=chooseDestination(rng);
      v.currentIntersectionId=chooseOrigin(rng);
      while(v.currentIntersectionId==v.destinationIntersectionId) v.currentIntersectionId=chooseOrigin(rng);
      v.positionOnRoad=0;
      v.spawnTime=ctime+1;
     

  }

  void oneStep() // de schimbat si in cazul in care vehiculul nu apare la multiplu de tstep
  {
      cout << ctime << '\n';
     for(int i=0; i<vehicles.size(); i++)
        {
         int ok=0;   
         vehicle v=vehicles[i];  
         v.currentRoadId = shortPaths[v.currentIntersectionId][v.destinationIntersectionId]; 
         line currentRoad = city.getLine(v.currentRoadId);
         
         if(v.spawnTime < ctime && ctime-v.spawnTime > 1e-5)
            {
              ok=1;
              v.positionOnRoad += tstep * currentRoad.maxspeed / currentRoad.lg;
              
              if(v.positionOnRoad >=1)
              {
                
                if(currentRoad.to == v.destinationIntersectionId) 
                {
                    node i1 = city.getIntersection(v.currentIntersectionId);
                    node i2 = city.getIntersection(currentRoad.to);
                    cout << v.id << ", roadid: " << currentRoad.id << ": " << i2.x << "/" << i2.y << '\n'; 
                    ok=0; 
                    reinitializeVehicle(vehicles[i]);
                } // to be implemented for large number of vehicles that appear and disappear
                else
                {
                float ftime= (v.positionOnRoad - 1) * currentRoad.lg / currentRoad.maxspeed;
                    
                v.currentIntersectionId= currentRoad.to;
                currentRoad = city.getLine(shortPaths[v.currentIntersectionId][v.destinationIntersectionId]);
                
                v.positionOnRoad = ftime * currentRoad.maxspeed / currentRoad.lg;
                
                }

              }
            }   
         
        
     if(ok){   
            node i1= city.getIntersection(v.currentIntersectionId);
            node i2 = city.getIntersection(currentRoad.to);
         
            cout << v.id << ", roadid: " << currentRoad.id <<  ": " << i1.x * (1-v.positionOnRoad) + i2.x * v.positionOnRoad << "/" << i1.y * (1-v.positionOnRoad) + i2.y * v.positionOnRoad << '\n';
               
            vehicles[i]=v; 
        }
         
        
        
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
   
   Simulation sim(42);
   
   
   sim.readCity();
   sim.setTime(0.0f,0.1f);
   sim.initializeShortestPaths();
   sim.initializeWeights(string(PROJECT_PATH) + "/data/demand.csv");
   
   sim.initializeVehicles();
    
   for(int i=1; i<=steps; i++)
    {
        sim.oneStep();
    }
    
    return 0;
}
