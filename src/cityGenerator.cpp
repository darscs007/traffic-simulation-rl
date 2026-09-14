#include <fstream>
#include <random>
#include "simulationConfig.h"
#include "cityTypes.h"
#include <vector>
#include <string>
#include "trafficConstraints.h"
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

std::ofstream g(std::string(PROJECT_PATH) + "/data//intersections.csv");  
std::ofstream h(std::string(PROJECT_PATH) + "/data/roads.csv");
std::ofstream p(std::string(PROJECT_PATH)+"/data/demand.csv");

simulationConfig config;

struct genConfig
{
 int noIntersections = 1000;
 int maxNoRoads = 1700; //with limits >=n-1, but the generator may not reach this number, <= 3n-3-h
 int halfLength =3000; // all intersections are in the square
 int seed=59;
 int nodeCandidates = 10;

 int minSpeed=5;
 int maxSpeed=40;

 float mindist = 12 * (config.CAR_LENGTH+config.SAFETY_GAP); // the minimum distance between 2 intersections (minimum road length) 
 float packness = 3.5f; // the higher the more packed the city is (here the stddev is 1/3 of radius, which means hat 99.73% of values would be in the radius range)
 int maximumRejection=70;
 float chanceBidirectional=0.8f;

 float triangleRetention=0.2f;
 float minReductionFactor = 1.4f; //used to check if a triangle is close to degenerate
 float baseInterest=0.0f;
 float baseDeparture=0.0f;
 float centralFactor=0.8f;
 float radialityFactor=2.0f;
 float extConnectFactor=0.1f;
 float intConnectFactor=0.1f;
 float randFactor=0.1f;

 bool moreRandom=0;
 bool generateCircular=1;
}configuration1;

class cityGenerator
{
private:
genConfig gendata;
std::mt19937 rng;
std::vector<std::vector<std::vector<int>>> nodeGrid;
std::vector<std::vector<std::vector<int>>> roadGrid;
std::vector<node> nodes;
std::vector<line> roads;
std::vector<line> distinctRoads;
std::vector<std::vector<std::vector<int>>> candidatesByNode; // and levels
std::unordered_set<std::uint64_t> isGenerated;
std::uniform_real_distribution<float> speedFactor{0.0f, 1.0f};
std::uniform_real_distribution<float> triangleRejectionRand{0.0f, 1.0f};
std::vector<std::vector<int>> RcandidatesByNode;
std::vector<std::vector<int>> adjExtList;
std::vector<std::vector<int>> adjIntList;
std::vector<std::vector<int>> adjPhysList;
std::normal_distribution<float> dist;
std::normal_distribution<float> disty;
std::uniform_real_distribution<float> angle;
std::normal_distribution<float> length;
std::uniform_real_distribution<float> isBidirectional{0.0f, 1.0f};
std::vector<int> lastRoadCheck;
int currentCheckNo=0;

int maxIntDeg=-1;
int maxExtDeg=-1;

struct triangleInfo 
{
    int count = 0;
    int minDist = traffic::INF;
    int minDistNode = -1;
};

std::uint64_t roadKey(int a, int b) 
{
 int first=std::min(a,b);
 int second=std::max(a,b);

 return (static_cast<std::uint64_t>(first) << 32 | static_cast<std::uint64_t>(second)); //binary key, 64 bits
}

int attributeSpeed(const node& start, const node& end)
{
 return gendata.minSpeed+(0.7f*sqrt(((start.x+end.x)*(start.x+end.x)/4+(start.y+end.y)*(start.y+end.y)/4)/(2*gendata.halfLength*gendata.halfLength)) + 0.3f * speedFactor(rng)) * (gendata.maxSpeed-gendata.minSpeed);
}

int orientation(const node& A, const node &B, const node&C)
{
  float res=(B.x-A.x)*(C.y-A.y)-(B.y-A.y)*(C.x-A.x);
  if(res > 0) return 1;
  else if(res < 0) return -1;
  return 0;

}

bool roadsIntersect(const node& A, const node&B, const node&C, const node&D) //AB and CD
{
if(orientation(A,B,C) * orientation(A,B,D) <= 0 && orientation(C,D,A)*orientation(C,D,B) <= 0) return 1;
return 0;

}

bool checkRoad(const node& startN, const node& endN)
{
  currentCheckNo++;
  
  int mstart=floor((gendata.halfLength-startN.y)/gendata.mindist);
  int nstart=floor((startN.x+gendata.halfLength)/gendata.mindist);
  int mend=floor((gendata.halfLength-endN.y)/gendata.mindist);
  int nend=floor((endN.x+gendata.halfLength)/gendata.mindist);

  int dirX, dirY;
  if(startN.x - endN.x <= 0 && startN.y - endN.y <= 0) {dirY=-1; dirX=1;}
  else if(startN.x - endN.x <= 0 && startN.y - endN.y > 0) {dirY=1; dirX=1;}
  else if(startN.x - endN.x > 0 && startN.y - endN.y > 0) {dirY=1;  dirX=-1;}
  else {dirY=-1; dirX=-1;}

  float nextX,nextY;
  if(dirX==-1)nextX=nstart*gendata.mindist-gendata.halfLength;
  else nextX=(nstart+1)*gendata.mindist - gendata.halfLength;
  
  if(dirY==-1) nextY = gendata.halfLength - mstart*gendata.mindist;
  else nextY = gendata.halfLength - (1+mstart)*gendata.mindist;


  while(mstart != mend || nstart != nend)
  {
    for(int i=0;i < roadGrid[mstart][nstart].size(); i++)
     {
      const line& intersRoad=distinctRoads[roadGrid[mstart][nstart][i]];
      if(lastRoadCheck[intersRoad.id] ==currentCheckNo) continue;
      
      if(intersRoad.from==startN.id  || intersRoad.from==endN.id  || intersRoad.to==startN.id  || intersRoad.to==endN.id)
      continue;
      
      lastRoadCheck[intersRoad.id]=currentCheckNo;

      if(roadsIntersect(startN,endN, nodes[intersRoad.from], nodes[intersRoad.to]))
          return 0;
     }


    if((endN.x-startN.x) == 0) {mstart+= dirY; nextY-=dirY*gendata.mindist;}
    else if(endN.y-startN.y == 0) {nstart+=dirX; nextX+=dirX*gendata.mindist;}
    else
    if((nextX-startN.x)/(endN.x-startN.x) > (nextY-startN.y)/(endN.y-startN.y)) {mstart+= dirY;nextY-=dirY*gendata.mindist;}
    else { nstart+=dirX; nextX+=dirX*gendata.mindist;}

   

  } 

for(int i=0;i < roadGrid[mstart][nstart].size(); i++)
     {
      const line& intersRoad=distinctRoads[roadGrid[mstart][nstart][i]];
      
      if(lastRoadCheck[intersRoad.id] ==currentCheckNo) continue;
      
      if(intersRoad.from==startN.id  || intersRoad.from==endN.id  || intersRoad.to==startN.id  || intersRoad.to==endN.id)
      continue;

      lastRoadCheck[intersRoad.id]=currentCheckNo;

      if(roadsIntersect(startN,endN, nodes[intersRoad.from], nodes[intersRoad.to]))
          return 0;
     }  



return 1;
}

void generateCandidates()
{

candidatesByNode.clear();
candidatesByNode.resize(nodes.size());

for(int i=0; i<nodes.size(); i++)
{
  int mstart=floor((gendata.halfLength-nodes[i].y)/gendata.mindist);
  int nstart=floor((nodes[i].x+gendata.halfLength)/gendata.mindist);
  
  bool levelCreated=0;
  bool notReachedLimit=1;
  
  int candidateCount=0;
  for(int j=0; j<nodeGrid[mstart][nstart].size(); j++)
    if(i != nodeGrid[mstart][nstart][j] && isGenerated.find(roadKey(std::min(i,nodeGrid[mstart][nstart][j]), std::max(i, nodeGrid[mstart][nstart][j]))) == isGenerated.end()) 
    {
      if(!levelCreated){ candidatesByNode[i].emplace_back(); levelCreated=1;}
      candidatesByNode[i][0].push_back(nodeGrid[mstart][nstart][j]);
      candidateCount++;
      if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
    }

  int startOffset=1;
  
  while(candidateCount < gendata.nodeCandidates && startOffset < 2*gendata.halfLength/gendata.mindist)
  {
    
    bool levelCreated=0;

    if(mstart-startOffset>=0) 
      for(int j=nstart-startOffset; j<nstart+startOffset && notReachedLimit; j++)
        if(j>=0 && j < nodeGrid[0].size())
          for(int e=0; e < nodeGrid[mstart-startOffset][j].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[mstart-startOffset][j][e]), std::max(i, nodeGrid[mstart-startOffset][j][e])))== isGenerated.end())
             {
              if(!levelCreated){candidatesByNode[i].emplace_back(); levelCreated=1;}
              candidatesByNode[i].back().push_back(nodeGrid[mstart-startOffset][j][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
             
            }

    if(nstart+startOffset<nodeGrid[0].size()) 
      for(int j=mstart-startOffset; j<mstart+startOffset && notReachedLimit; j++)
        if(j>=0 && j < nodeGrid.size())
          for(int e=0; e < nodeGrid[j][nstart+startOffset].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[j][nstart+startOffset][e]), std::max(i, nodeGrid[j][nstart+startOffset][e])))== isGenerated.end())
             {
              if(!levelCreated){candidatesByNode[i].emplace_back(); levelCreated=1;}
              candidatesByNode[i].back().push_back(nodeGrid[j][nstart+startOffset][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
            
            }     
            
    if(mstart+startOffset < nodeGrid.size()) 
      for(int j=nstart+startOffset; j>nstart-startOffset && notReachedLimit; j--)
        if(j>=0 && j < nodeGrid[0].size())
          for(int e=0; e < nodeGrid[mstart+startOffset][j].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[mstart+startOffset][j][e]), std::max(i, nodeGrid[mstart+startOffset][j][e])))== isGenerated.end())
             {
              if(!levelCreated){candidatesByNode[i].emplace_back(); levelCreated=1;}              
              candidatesByNode[i].back().push_back(nodeGrid[mstart+startOffset][j][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
             
            }  

    if(nstart-startOffset>=0) 
      for(int j=mstart+startOffset; j > mstart-startOffset && notReachedLimit; j--)
        if(j>=0 && j < nodeGrid.size())
          for(int e=0; e < nodeGrid[j][nstart-startOffset].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[j][nstart-startOffset][e]), std::max(i, nodeGrid[j][nstart-startOffset][e])))== isGenerated.end())
             {
              if(!levelCreated){candidatesByNode[i].emplace_back(); levelCreated=1;}
              candidatesByNode[i].back().push_back(nodeGrid[j][nstart-startOffset][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
             
            } 
            
      startOffset++;
  }

  std::reverse(candidatesByNode[i].begin(), candidatesByNode[i].end()); // for easier deletion of closer levels (they are prioritised)
}


}

void generateCandidatesRandom()
{
RcandidatesByNode.clear();
RcandidatesByNode.resize(nodes.size());

for(int i=0; i<nodes.size(); i++)
{
  int mstart=floor((gendata.halfLength-nodes[i].y)/gendata.mindist);
  int nstart=floor((nodes[i].x+gendata.halfLength)/gendata.mindist);
  
  bool notReachedLimit=1;
  
  int candidateCount=0;
  for(int j=0; j<nodeGrid[mstart][nstart].size(); j++)
    if(i != nodeGrid[mstart][nstart][j] && isGenerated.find(roadKey(std::min(i,nodeGrid[mstart][nstart][j]), std::max(i, nodeGrid[mstart][nstart][j]))) == isGenerated.end()) 
    {
      RcandidatesByNode[i].push_back(nodeGrid[mstart][nstart][j]);
      candidateCount++;
      if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
    }

  int startOffset=1;
  
  while(candidateCount < gendata.nodeCandidates && startOffset < 2*gendata.halfLength/gendata.mindist)
  {
    
   

    if(mstart-startOffset>=0) 
      for(int j=nstart-startOffset; j<nstart+startOffset && notReachedLimit; j++)
        if(j>=0 && j < nodeGrid[0].size())
          for(int e=0; e < nodeGrid[mstart-startOffset][j].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[mstart-startOffset][j][e]), std::max(i, nodeGrid[mstart-startOffset][j][e])))== isGenerated.end())
             {
              RcandidatesByNode[i].push_back(nodeGrid[mstart-startOffset][j][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
             
            }

    if(nstart+startOffset<nodeGrid[0].size()) 
      for(int j=mstart-startOffset; j<mstart+startOffset && notReachedLimit; j++)
        if(j>=0 && j < nodeGrid.size())
          for(int e=0; e < nodeGrid[j][nstart+startOffset].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[j][nstart+startOffset][e]), std::max(i, nodeGrid[j][nstart+startOffset][e])))== isGenerated.end())
             {
              RcandidatesByNode[i].push_back(nodeGrid[j][nstart+startOffset][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
            
            }     
            
    if(mstart+startOffset < nodeGrid.size()) 
      for(int j=nstart+startOffset; j>nstart-startOffset && notReachedLimit; j--)
        if(j>=0 && j < nodeGrid[0].size())
          for(int e=0; e < nodeGrid[mstart+startOffset][j].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[mstart+startOffset][j][e]), std::max(i, nodeGrid[mstart+startOffset][j][e])))== isGenerated.end())
             {
                          
              RcandidatesByNode[i].push_back(nodeGrid[mstart+startOffset][j][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
             
            }  

    if(nstart-startOffset>=0) 
      for(int j=mstart+startOffset; j > mstart-startOffset && notReachedLimit; j--)
        if(j>=0 && j < nodeGrid.size())
          for(int e=0; e < nodeGrid[j][nstart-startOffset].size(); e++)
            {
             if(isGenerated.find(roadKey(std::min(i,nodeGrid[j][nstart-startOffset][e]), std::max(i, nodeGrid[j][nstart-startOffset][e])))== isGenerated.end())
             {
              RcandidatesByNode[i].push_back(nodeGrid[j][nstart-startOffset][e]);
              candidateCount++;
              if(candidateCount==gendata.nodeCandidates){notReachedLimit=0; break;}
             }
             
            } 
            
      startOffset++;
  }

  std::reverse(RcandidatesByNode[i].begin(), RcandidatesByNode[i].end()); // for easier deletion of closer levels (they are prioritised)
}
}

bool validateGrid(int m, int n, float x, float y)
    {
     for(int i=0; i<nodeGrid[m][n].size(); i++)
            if((nodes[nodeGrid[m][n][i]].x-x)*(nodes[nodeGrid[m][n][i]].x-x) + (nodes[nodeGrid[m][n][i]].y-y)*(nodes[nodeGrid[m][n][i]].y-y) < gendata.mindist*gendata.mindist) 
            return 0;

      return 1;      
    }    

bool generateNode()
    {
    float x,y;
     if(!gendata.generateCircular)
     {
     x= dist(rng);
     while(x > gendata.halfLength || x < 0-gendata.halfLength) x= dist(rng);
     
     y=disty(rng);
     while(y > gendata.halfLength || y < 0-gendata.halfLength) y=disty(rng);
     }
     else
     {
      float A= angle(rng);
      float cosA=std::cos(A);
      float l=length(rng);
      while(abs(l*cosA) > gendata.halfLength || abs(l*std::sqrt(1.0f-cosA*cosA)) > gendata.halfLength) l=length(rng);

      x=l*cosA;
      y=l*std::sqrt(1.0f - cosA*cosA);
     }
     int n,m;
     m=floor((gendata.halfLength-y)/gendata.mindist);
     n=floor((x+gendata.halfLength)/gendata.mindist);
     //[m][n]

     if(!validateGrid(m,n,x,y)) return 0;
     if(m && !validateGrid(m-1,n,x,y))return 0;
     if(m<nodeGrid.size()-1 && !validateGrid(m+1,n,x,y)) return 0;
     if(n && !validateGrid(m,n-1,x,y)) return 0;
     if(n<nodeGrid[0].size()-1 && !validateGrid(m,n+1,x,y)) return 0;
     if(n && m && !validateGrid(m-1,n-1,x,y) ) return 0;
     if(m && n<nodeGrid[0].size()-1 && !validateGrid(m-1,n+1,x,y)) return 0;
     if(m < nodeGrid.size()-1 && n < nodeGrid[0].size()-1 && !validateGrid(m+1,n+1,x,y)) return 0;
     if(n && m < nodeGrid.size()-1 && !validateGrid(m+1,n-1,x,y)) return 0;

     
     int nodeId = static_cast<int>(nodes.size());
     int externalSpeed=gendata.minSpeed+(sqrt((x*x+y*y)/(2*gendata.halfLength*gendata.halfLength)) *  0.6f + 0.4f *speedFactor(rng))*(gendata.maxSpeed-gendata.minSpeed) ;

     nodes.push_back({x,y,externalSpeed, nodeId});
     nodeGrid[m][n].push_back(nodeId);

     return 1;
    }

void addPhysList(int startId, int endId)
{
std::vector<int>::iterator insertPos = std::lower_bound(adjPhysList[endId].begin(),adjPhysList[endId].end(),startId);
adjPhysList[endId].insert(insertPos,startId);
} 

int distance(const node& n1, const node& n2)
{
  return std::sqrt((n1.x-n2.x)*(n1.x-n2.x) + (n1.y-n2.y)*(n1.y-n2.y));

}

triangleInfo willBeTriangle(int id1, int id2)
{
  int minDist=traffic::INF;
  int minDistNode=-1;
  int count =0;
  int i=0,j=0;
  while(i<adjPhysList[id1].size() && j < adjPhysList[id2].size())
    if(adjPhysList[id1][i] < adjPhysList[id2][j]) i++;
    else if(adjPhysList[id1][i] > adjPhysList[id2][j]) j++;
    else 
    {
      
      count++;
      if(distance(nodes[id1],nodes[adjPhysList[id1][i]])+distance(nodes[id2],nodes[adjPhysList[id1][i]]) < minDist) 
      {
       minDist=distance(nodes[id1],nodes[adjPhysList[id1][i]])+distance(nodes[id2],nodes[adjPhysList[id1][i]]);
       minDistNode = adjPhysList[id1][i];
      }
      i++; j++;
    }

  return {count,minDist,minDistNode};

}

void mst()
{
  std::vector<int> bestParent(nodes.size(),0);
  std::vector<float> bestDistanceSq(nodes.size(),traffic::INF);
  
  for(int i=1; i<nodes.size(); i++)
    {
      bestParent[i]=0;
      bestDistanceSq[i]=(nodes[0].x-nodes[i].x)*(nodes[0].x-nodes[i].x)+(nodes[0].y-nodes[i].y)*(nodes[0].y-nodes[i].y);
    }
  
  for(int i=1; i< nodes.size(); i++)
  {
    int minNode; float minDistance= traffic::INF;
    for(int j=1; j< nodes.size(); j++)
        if(bestDistanceSq[j]!=-1 && bestDistanceSq[j] < minDistance)
        {
          minNode=j;
          minDistance=bestDistanceSq[j];
        }

    int minDist = std::sqrt(minDistance);

    roads.push_back({minNode,bestParent[minNode],minDist,10,static_cast<int>(roads.size())});
    roads.push_back({bestParent[minNode],minNode,minDist,10,static_cast<int>(roads.size())});

    adjExtList[minNode].push_back(bestParent[minNode]);
    adjIntList[bestParent[minNode]].push_back(minNode);
    adjExtList[bestParent[minNode]].push_back(minNode);
    adjIntList[minNode].push_back(bestParent[minNode]);
    
    lastRoadCheck.push_back(-1);

    addPhysList(bestParent[minNode],minNode);
    addPhysList(minNode,bestParent[minNode]);    

    distinctRoads.push_back({std::min(minNode,bestParent[minNode]),std::max(bestParent[minNode],minNode),minDist,10,static_cast<int>(distinctRoads.size())});
    isGenerated.insert(roadKey(std::min(minNode,bestParent[minNode]),std::max(bestParent[minNode],minNode)));

    bestDistanceSq[minNode]=-1;

    for(int j=1; j < nodes.size(); j++)
      if( bestDistanceSq[j] > (nodes[minNode].x-nodes[j].x)*(nodes[minNode].x-nodes[j].x) + (nodes[minNode].y-nodes[j].y)*(nodes[minNode].y-nodes[j].y))
        {
          bestDistanceSq[j]=(nodes[minNode].x-nodes[j].x)*(nodes[minNode].x-nodes[j].x) + (nodes[minNode].y-nodes[j].y)*(nodes[minNode].y-nodes[j].y);
          bestParent[j]=minNode;
          
        }
  }


}

void createRoad(int nodeId, int chosenId)
{
          triangleInfo res=willBeTriangle(nodeId,chosenId);
          if(res.minDistNode!=-1 && (triangleRejectionRand(rng) > pow(gendata.triangleRetention,res.count) || res.minDist < gendata.minReductionFactor * distance(nodes[nodeId],nodes[chosenId])))
          return ;
          
          markRoadGrid(nodes[nodeId], nodes[chosenId], distinctRoads.size());
          float distance=std::sqrt((nodes[nodeId].x-nodes[chosenId].x)*(nodes[nodeId].x-nodes[chosenId].x)+(nodes[nodeId].y-nodes[chosenId].y)*(nodes[nodeId].y-nodes[chosenId].y));
          distinctRoads.push_back({std::min(nodeId,chosenId),std::max(nodeId,chosenId),(int)distance,10,static_cast<int>(distinctRoads.size())});
          isGenerated.insert(roadKey(nodeId, chosenId));

          int chosenSpeed=attributeSpeed(nodes[nodeId],nodes[chosenId]);
          roads.push_back({nodeId,chosenId,(int)distance,chosenSpeed,static_cast<int>(roads.size())});
          adjExtList[nodeId].push_back(chosenId);
          adjIntList[chosenId].push_back(nodeId);

          addPhysList(nodeId,chosenId);
          addPhysList(chosenId,nodeId);  

          lastRoadCheck.push_back(-1);

          if(isBidirectional(rng) * (1.0f - distance/gendata.halfLength) < gendata.chanceBidirectional)
            {
              roads.push_back({chosenId,nodeId,(int)distance,chosenSpeed,static_cast<int>(roads.size())});
              adjExtList[chosenId].push_back(nodeId);
              adjIntList[nodeId].push_back(chosenId);
            }

}

void connectMore()
{
  std::vector<int>nodeOrder;
  std::uniform_int_distribution<> chooseNode(0,nodes.size()-1);


  for(int i=0; i<distinctRoads.size(); i++)
    isGenerated.insert(roadKey(std::min(distinctRoads[i].from, distinctRoads[i].to), std::max(distinctRoads[i].from, distinctRoads[i].to)));

  for(int i=0; i<nodes.size(); i++) nodeOrder.push_back(i);

  generateCandidates();
 
  while(nodeOrder.size() && distinctRoads.size() < gendata.maxNoRoads)
  {
    std::shuffle(nodeOrder.begin(),nodeOrder.end(),rng);

    for(int i=0; i<nodeOrder.size() && distinctRoads.size() < gendata.maxNoRoads; i++)
    {
      int nodeId=nodeOrder[i];

      if(!candidatesByNode[nodeId].empty() && !candidatesByNode[nodeId].back().empty())
      std::shuffle(candidatesByNode[nodeId].back().begin(),candidatesByNode[nodeId].back().end(), rng);

      bool chosenCandidate=0;
      while(!chosenCandidate && candidatesByNode[nodeId].size())
      {
        while(candidatesByNode[nodeId].back().size() && isGenerated.find(roadKey(std::min(nodeId,candidatesByNode[nodeId].back().back()), std::max(nodeId,candidatesByNode[nodeId].back().back()))) != isGenerated.end())
        candidatesByNode[nodeId].back().pop_back();

        if(!candidatesByNode[nodeId].back().size()) candidatesByNode[nodeId].pop_back();
        else chosenCandidate=1;


      }

      //chosen is candidatedByNode[i].back().back()
      if(chosenCandidate)
      {
      int chosenId=candidatesByNode[nodeId].back().back();
      candidatesByNode[nodeId].back().pop_back();
      if(!candidatesByNode[nodeId].back().size())
        candidatesByNode[nodeId].pop_back();

      if(checkRoad(nodes[nodeId],nodes[chosenId]))
        {
          createRoad(nodeId, chosenId);
          
        }
     }
    
     if(!candidatesByNode[nodeId].size())
        {
          std::swap(nodeOrder[i], nodeOrder.back());
          nodeOrder.pop_back();
          i--;
        }
    }

  }
}

void connectMoreRandom()
{
  std::vector<int>nodeOrder;
  std::uniform_int_distribution<> chooseNode(0,nodes.size()-1);
  std::uniform_real_distribution<float> isBidirectional(0.0f, 1.0f);

  for(int i=0; i<distinctRoads.size(); i++)
    isGenerated.insert(roadKey(std::min(distinctRoads[i].from, distinctRoads[i].to), std::max(distinctRoads[i].from, distinctRoads[i].to)));

  for(int i=0; i<nodes.size(); i++) nodeOrder.push_back(i);

  generateCandidatesRandom();


  
  while(nodeOrder.size() && distinctRoads.size() < gendata.maxNoRoads)
  {
    
    std::shuffle(nodeOrder.begin(),nodeOrder.end(),rng);

    for(int i=0; i<nodeOrder.size() && distinctRoads.size() < gendata.maxNoRoads; i++)
    {
      int nodeId=nodeOrder[i];

      if(!RcandidatesByNode[nodeId].empty())
      std::shuffle(RcandidatesByNode[nodeId].begin(),RcandidatesByNode[nodeId].end(), rng);

      bool chosenCandidate=0;
      while(!chosenCandidate && RcandidatesByNode[nodeId].size())
        if(isGenerated.find(roadKey(std::min(nodeId,RcandidatesByNode[nodeId].back()), std::max(nodeId,RcandidatesByNode[nodeId].back()))) != isGenerated.end())
            RcandidatesByNode[nodeId].pop_back();
        else chosenCandidate=1; 

      //chosen is candidatedByNode[i].back().back()
      if(chosenCandidate)
      {
      int chosenId=RcandidatesByNode[nodeId].back();
      RcandidatesByNode[nodeId].pop_back();

      if(checkRoad(nodes[nodeId],nodes[chosenId]))
        {
          createRoad(nodeId, chosenId);
          
        }
    

      
      }

      if(!RcandidatesByNode[nodeId].size())
      {
        std::swap(nodeOrder[i],nodeOrder.back());
        nodeOrder.pop_back();
        i--;  
      }

    }

  }

}

void markRoadGrid(const node& startN, const node& endN, int id)
{
  int mstart=floor((gendata.halfLength-startN.y)/gendata.mindist);
  int nstart=floor((startN.x+gendata.halfLength)/gendata.mindist);
  int mend=floor((gendata.halfLength-endN.y)/gendata.mindist);
  int nend=floor((endN.x+gendata.halfLength)/gendata.mindist);

  int dirX,dirY;
  if(startN.x - endN.x <= 0 && startN.y - endN.y <= 0) {dirY=-1; dirX=1;}
  else if(startN.x - endN.x <= 0 && startN.y - endN.y > 0) {dirY=1; dirX=1;}
  else if(startN.x - endN.x > 0 && startN.y - endN.y > 0) {dirY=1; dirX=-1;}
  else {dirY=-1; dirX=-1;}

  
  float nextX,nextY;
  if(dirX==-1)nextX=nstart*gendata.mindist-gendata.halfLength;
  else nextX=(nstart+1)*gendata.mindist - gendata.halfLength;
  
  if(dirY==-1) nextY = gendata.halfLength - mstart*gendata.mindist;
  else nextY = gendata.halfLength - (1+mstart)*gendata.mindist;

  roadGrid[mstart][nstart].push_back(id);

  while(mstart != mend || nstart != nend)
  {
    if((endN.x-startN.x) == 0) {mstart+= dirY; nextY-=dirY*gendata.mindist;}
    else if(endN.y-startN.y == 0) {nstart+=dirX; nextX+=dirX*gendata.mindist;}
    else
    if((nextX-startN.x)/(endN.x-startN.x) > (nextY-startN.y)/(endN.y-startN.y)) {mstart+= dirY;nextY-=dirY*gendata.mindist;}
    else { nstart+=dirX; nextX+=dirX*gendata.mindist;}

    roadGrid[mstart][nstart].push_back(id);
  }

}

void attributeWeights()
{
std::uniform_real_distribution<float> randWeight{0.0f,1.0f};

  for(int i=0; i<nodes.size(); i++)
  {
    if(maxExtDeg < static_cast<int>(adjExtList[i].size()))
      maxExtDeg=adjExtList[i].size();

    if(maxIntDeg < static_cast<int>(adjIntList[i].size()))
      maxIntDeg=adjIntList[i].size();
  }

p << "index,interest_weight,departure_weight\n";
for(int i=0; i<nodes.size(); i++)
{
float depWeight, destWeight;

float radiality=std::sqrt(nodes[i].x*nodes[i].x+nodes[i].y*nodes[i].y)/(std::sqrt(2)*gendata.halfLength);

depWeight=gendata.baseDeparture+ std::pow(radiality,3)*gendata.radialityFactor + (float)adjExtList[i].size()/maxExtDeg * gendata.extConnectFactor + randWeight(rng) * gendata.randFactor;
destWeight=gendata.baseInterest+ std::pow((1-radiality),3)*gendata.centralFactor +(float)adjIntList[i].size()/maxIntDeg * gendata.intConnectFactor + randWeight(rng) * gendata.randFactor;

p << i << "," << (int)(destWeight*100) << "," << (int)(depWeight*100) <<'\n';

}



}

public:
    explicit cityGenerator(const genConfig data)
    : gendata(data), rng(data.seed)
    {}

    void generate()
    {
      nodeGrid.resize(std::floor(2*gendata.halfLength/ gendata.mindist)+1, std::vector<std::vector<int>>(std::floor(2*gendata.halfLength/ gendata.mindist)+1));
      roadGrid.resize(std::floor(2*gendata.halfLength/ gendata.mindist)+1, std::vector<std::vector<int>>(std::floor(2*gendata.halfLength/ gendata.mindist)+1));

      if(gendata.generateCircular) 
      {
      angle = std::uniform_real_distribution<float> {0.0f, std::acos(-1.0f)}; 
      length=std::normal_distribution<float> {0.0f,(float)gendata.halfLength/ gendata.packness};
      }
      else 
      {
      dist=std::normal_distribution<float> {0.0f, (float)gendata.halfLength/ gendata.packness}; 
      disty=std::normal_distribution<float> {0.0f, (float)gendata.halfLength/ gendata.packness};
      }

      int currentRejection=0, currentGenerationId=0;
      while(currentGenerationId != gendata.noIntersections && currentRejection < gendata.maximumRejection)
        if(generateNode()){currentRejection=0;currentGenerationId++;}
        else currentRejection++;
      
     
      g << "x,y,external_speed,id\n";
      for(int i=0; i<nodes.size(); i++)
        g << nodes[i].x << "," << nodes[i].y << "," << nodes[i].externalSpeed << "," << nodes[i].id << '\n';


      adjExtList.resize(nodes.size());
      adjIntList.resize(nodes.size());
      adjPhysList.resize(nodes.size());

      mst();
      
      for(int i=0; i<distinctRoads.size(); i++)
         markRoadGrid(nodes[distinctRoads[i].from], nodes[distinctRoads[i].to],i);
      


      
      h << "from,to,lg,maxspeed,id\n";
      for(int i=0; i<roads.size(); i++)
      {

        if(i%2==0)
        {
          roads[i].maxspeed= attributeSpeed(nodes[roads[i].from],nodes[roads[i].to]); 
          distinctRoads[i/2].maxspeed=roads[i].maxspeed;
        }
        else roads[i].maxspeed=roads[i-1].maxspeed;

      }

      gendata.nodeCandidates=std::min(gendata.nodeCandidates, (int)std::ceil(static_cast<float>(nodes.size())/4));

      if(!gendata.moreRandom)connectMore();
      else connectMoreRandom();

      for(int i=0; i<roads.size();i++)
       h << roads[i].from << "," << roads[i].to << "," << roads[i].lg << "," << roads[i].maxspeed << "," << roads[i].id << '\n';

      attributeWeights();
    }


};


int main()
{
    cityGenerator gen1(configuration1);

    gen1.generate();

    return 0;
}

