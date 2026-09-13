#pragma once

#include <cstdint>
#include <vector>

struct traceHeader
{
std::uint32_t magic;
std::uint32_t version;
std::uint32_t frameCount;
std::uint32_t roadCount;
};

inline constexpr std::uint32_t TRACE_VERSION =2;
inline constexpr std::uint32_t STATS_MAGIC=0x53544154;
inline constexpr std::uint32_t CONGESTION_MAGIC=0x434F4E47;
inline constexpr std::uint32_t VEHICLES_MAGIC=0x56454849;

struct statsFormat
{
std::uint32_t noTrips;
double extAvg, extW, intAvg, intW, totAvg, totW;
std::uint32_t moving, stationary;
double fmoving, fstationary;
int active;
};


struct congestionFormat //documentation only
{
float occupancy;

};

struct detailedFrameHeader //documentation only
{
std::vector<std::int32_t> tlPhases;
std::uint32_t active;
};

struct detailedVehicle
{
std::uint32_t id;
std::uint32_t roadId;
float positionOnRoad;
std::uint32_t impatient;

};
