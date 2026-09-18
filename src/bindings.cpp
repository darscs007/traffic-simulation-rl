#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <fstream>
#include "Simulation.h"
#include "trafficLightSystem.h"
#include <utility>
#include "simulationConfig.h"
#include <filesystem>
#include <stdexcept>
#include "traceFormats.h"

namespace py = pybind11;

simulationConfig config;

std::ofstream noTrace0, noTrace1, noTrace2;
Simulation sim(config, noTrace0, noTrace1, noTrace2);

void startOutput()
{
    const std::filesystem::path output =
        std::filesystem::path(PROJECT_PATH) / "output";

    noTrace0.open(output / "level0" / "statistics.bin",
                  std::ios::binary | std::ios::trunc);
    noTrace1.open(output / "level1" / "congestion.bin",
                  std::ios::binary | std::ios::trunc);
    noTrace2.open(output / "level2" / "detailed.bin",
                  std::ios::binary | std::ios::trunc);

    if (!noTrace0 || !noTrace1 || !noTrace2)
        throw std::runtime_error("Could not open trace files");

    traceHeader statsHeader{STATS_MAGIC, TRACE_VERSION, config.steps, 0};
    traceHeader congestionHeader{
        CONGESTION_MAGIC,
        TRACE_VERSION,
        config.steps,
        static_cast<std::uint32_t>(sim.getNoRoads())
    };
    traceHeader detailedHeader{VEHICLES_MAGIC, TRACE_VERSION, config.steps, 0};

    noTrace0.write(reinterpret_cast<const char*>(&statsHeader), sizeof(statsHeader));
    noTrace1.write(reinterpret_cast<const char*>(&congestionHeader), sizeof(congestionHeader));

    if (config.detailedRendering)
        noTrace2.write(reinterpret_cast<const char*>(&detailedHeader), sizeof(detailedHeader));

}

void finishOutput()
{

    noTrace0.close();
    noTrace1.close();
    noTrace2.close();
}

void clearCallbacks()
{
    sim.setRLCallbacks(sendData{}, getActions{});
}

void prepare()
{
sim.readCity();
sim.setTime(0.0, 0.1);
sim.initializeShortestPaths();
sim.initializeWeights(config.cityDirectory + "demand.csv");
sim.configureTrafficLights();
sim.initializeVehicles();
}

void runEpisode()
{
    for (int step = 0; step < config.steps; ++step)
        sim.oneStep(step);
}

void registerCallbacks(sendData sender, getActions receiver)
{
    sim.setRLCallbacks(std::move(sender), std::move(receiver));
}

void reset(int i)
{
 sim.reset(i);
}

py::dict getConfig()
    {
    py::dict result;

    result["max_cars"] = config.maxcars;
    result["steps"] = config.steps;
    result["green_steps"] = config.defaultGreenSteps;

    return result;
    }

PYBIND11_MODULE(traffic_rl_native, m) {
    m.def("start_output", &startOutput);
    m.def("finish_output", &finishOutput);
    m.def("get_config", &getConfig);

    py::class_<roadObservation>(m, "RoadObservation")
    .def_readonly("detected_vehicles", &roadObservation::detectedVehicles)
    .def_readonly("capacity", &roadObservation::capacity);

    py::class_<stateTL>(m, "StateTL")
    .def_readonly("current_phase", &stateTL::currentPhase)
    .def_readonly("time_since_phase", &stateTL::timeSincePhase)
    .def_readonly("external_queue_size", &stateTL::externalQueueSize)
    .def_readonly("incoming_roads", &stateTL::incomingRoads)
    .def_readonly("outgoing_roads", &stateTL::outgoingRoads)
    .def_readonly("road_change_matrix", &stateTL::roadChangeMatrix)
    .def_readonly("adj_tl_phases", &stateTL::adjTLPhases)
    .def_readonly("adj_tl_times", &stateTL::adjTLTimes); 
    
    py::class_<controlDataRL>(m, "ControlDataRL")
    .def_readonly("states", &controlDataRL::states)
    .def_readonly("rewards", &controlDataRL::rewards)
    .def_readonly("step", &controlDataRL::step)
    .def_readonly("first_step", &controlDataRL::firstStep)
    .def_readonly("done", &controlDataRL::done);

m.def("prepare", &prepare);
m.def("set_callbacks", &registerCallbacks);
m.def("run_episode", &runEpisode);
m.def("clear_callbacks", &clearCallbacks);
m.def("reset", &reset);
}




