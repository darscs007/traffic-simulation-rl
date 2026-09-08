#pragma once

#include <vector>
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
    std::vector<int> forbiddenRoads; //rerouting to these roads took it to the same intersection, so it won't make the same mistakes
    bool isImpatient;
};