#pragma once

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