#include "Graph.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "trafficConstraints.h"
#include <queue>
#include <functional>

typedef std::pair<float,int> dPair;

    std::vector<std::string> Graph::validateRoad(int fromi, int toi, int lgi, int maxspeedi, int idi) const
    {
      std::vector<std::string> errors;

      if(nodeIndexMap.find(fromi) == nodeIndexMap.end()) 
          errors.push_back( "Invalid 'from' intersection id: " + std::to_string(fromi));
               
      if(nodeIndexMap.find(toi) == nodeIndexMap.end()) 
          errors.push_back("Invalid 'to' intersection id: " + std::to_string(toi));
       
      if(fromi == toi) 
           errors.push_back("Road cannot connect the same intersection: " + std::to_string(fromi));

      if(lgi <= 0) 
           errors.push_back("Invalid road length: " + std::to_string(lgi)); 

      if(maxspeedi <= 0) 
        errors.push_back("Invalid road max speed: " + std::to_string(maxspeedi));  
                   
      if(idi < 0) 
        errors.push_back("Invalid road id: " + std::to_string(idi));  

      if(lineIndexMap.find(idi) != lineIndexMap.end()) 
          errors.push_back("Duplicate road id found: " + std::to_string(idi)); 

      if(idi != lines.size())
      errors.push_back("Unconsecutive Road Id: "+ std::to_string(idi));

      return errors;
    }

    std::vector<std::string> Graph::validateIntersection(int externalSpeedi, int idi) const
    {
      std::vector<std::string> errors;
      
      if (nodeIndexMap.find(idi) != nodeIndexMap.end()) 
                errors.push_back("Duplicate intersection id found: " + std::to_string(idi));

      if(externalSpeedi <= 0)
      errors.push_back("Negative or zero external road speed: "+ std::to_string(externalSpeedi));
       
      if(idi != nodes.size())
      errors.push_back("Unconsecutive Intersection Id: "+ std::to_string(idi));

      return errors;
    }

    const line& Graph::getLine(int id) const
    {
        return lines[id];

    }

    const node& Graph::getIntersection(int id) const
    {
      return nodes[id];

    }

    void Graph::addNode(float x, float y, int externalSpeed,int id)
       {
        nodes.push_back({x,y,externalSpeed,id});
        nodeIndexMap[id] = nodes.size()-1; // store the position of the intersection in the vector
        if(adjList.size() <= id)
        {
            adjList.resize(id+1);
        }
       }

    void Graph::addLine(int from, int to, int lg, int maxspeed, int id)
    { 
        lines.push_back({from, to, lg, maxspeed, id});
        lineIndexMap[id] = lines.size()-1; // store the position of the road in the vector
        adjList[from].push_back(lines.size()-1); // add vector index of the road in the adjacency list to know all the details of the roads that start from an intersection
    }

    void Graph::readIntersections(const std::string& fileName)
    {
        std::ifstream file(fileName);
        if(!file.is_open())
          throw std::runtime_error("Cannot open intersection data: " + fileName);
        
        std::string csvLine, x, y, externalSpeed,id;

        std::getline(file, csvLine); // antet: x,y,extspeed,id

        while (getline(file, csvLine)) {
            std::stringstream stream(csvLine);

            std::getline(stream, x, ',');
            std::getline(stream, y, ',');
            std::getline(stream,externalSpeed,',');
            std::getline(stream, id, ',');

            std::vector<std::string> errors=validateIntersection(stoi(externalSpeed), stoi(id));

            if(errors.size())
              throw std::invalid_argument("Invalid intersection entry: " + csvLine + " - "+ errors[0]);

            addNode(stof(x), stof(y), stoi(externalSpeed),stoi(id));
        }
    }

    void Graph::getShortestPath(std::vector<std::vector<int>> &a, std::vector<std::vector<int>> &b, int source, int destination, int &x)//calculates the successors
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
    
   void Graph::readRoads(const std::string& fileName)
    {
        std::ifstream file(fileName);
        if(!file.is_open())
         throw std::runtime_error("Cannot open road data: " + fileName);
        
        std::string csvLine, from, to, lg, maxspeed, id;

        std::getline(file, csvLine); // antet: from,to,lg,maxspeed,id

        while (std::getline(file, csvLine)) {
            std::stringstream stream(csvLine);

            std::getline(stream, from, ',');
            std::getline(stream, to, ',');
            std::getline(stream, lg, ',');
            std::getline(stream, maxspeed, ',');
            std::getline(stream, id, ',');
            int fromi=stoi(from);
            int toi=stoi(to);
            int lgi=stoi(lg);
            int maxspeedi=stoi(maxspeed);
            int idi=stoi(id);

            std::vector<std::string> errors= validateRoad(fromi,toi,lgi,maxspeedi,idi);
            if(errors.size())
              throw std::invalid_argument("Invalid road entry: " + csvLine + " - " + errors[0]);

            addLine(fromi, toi, lgi, maxspeedi, idi);
        }
    }

   void Graph::initializeDijkstraShortestPaths()
   {
    
    shortestPaths=std::vector<std::vector<int>> (nodes.size(), std::vector<int>(nodes.size(), -1));
    std::vector<std::vector<int>> successors(nodes.size(),std::vector<int>(nodes.size(), -1));

  
    for(int i=0;i<nodes.size();i++)
    {
        std::vector<float> d(nodes.size(), traffic::INF);
        std::priority_queue< dPair, std::vector<dPair>, std::greater<dPair> > pq;
      
        d[i]=0;
        pq.push(std::make_pair(0.0,i));

        while(!pq.empty())
        {
          float currentTime = pq.top().first;
          int last = pq.top().second;
          pq.pop();

          if (currentTime > d[last])
            continue;

          for(int j=0; j<adjList[last].size(); j++)
          {
            const line& cline=lines[adjList[last][j]];
            int nodeId=cline.to;
            float time= (float) cline.lg/cline.maxspeed;

            
            if( d[last]+time < d[nodeId])
            {
            d[nodeId]=d[last]+time;
            pq.push(std::make_pair(d[nodeId], nodeId));
            shortestPaths[i][nodeId] = adjList[last][j];  
            }

          }

        }

    }
    
    timeMatrix.clear();
    timeMatrix.resize(shortestPaths.size(),std::vector<float>(shortestPaths[0].size(),0)); // generate timeMatrix

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
    
   int Graph::getNoIntersections()const
   {
    return nodes.size();
   }

   int Graph::getNoRoads()const
   {
    return lines.size();
   }

   const std::vector<int>& Graph::getAdjRoads(int id) const // number of roads that start in the intersection with this id
   {
    return adjList[id];
   }

   void Graph::showTimeMatrix()const
   {
    for(int i=0; i<timeMatrix.size(); i++)
        {for(int j=0; j<timeMatrix[i].size(); j++) std::cout << timeMatrix[i][j] << " ";
         std::cout << '\n';
        }
   }

   float Graph::getTimeBetween(int i, int j)const
   {
    return timeMatrix[i][j];
   }

   int Graph::getNextRoadBetween(int i, int j)const
   {
    return shortestPaths[i][j];
   }
