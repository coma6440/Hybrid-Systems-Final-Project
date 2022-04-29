/* Standard Libraries */
#include <iostream>
#include <fstream>
#include <vector>

/* OMPL Libraries */
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

#include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
#include <ompl/control/planners/ltl/PropositionalDecomposition.h>
#include <ompl/control/planners/ltl/Automaton.h>
#include <ompl/control/planners/ltl/ProductGraph.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
#include <ompl/control/planners/ltl/LTLProblemDefinition.h>

#include <ompl/tools/benchmark/Benchmark.h>

/* Additional Libraries */
#include <json.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;

using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
using Vertex = oc::PropositionalTriangularDecomposition::Vertex;
using JSON = nlohmann::json;

// a decomposition is only needed for SyclopRRT and SyclopEST
// use TriangularDecomp
class MyDecomposition : public oc::PropositionalTriangularDecomposition
    {
        public:
        MyDecomposition(const ob::RealVectorBounds& bounds)
            : oc::PropositionalTriangularDecomposition(bounds) {}
        ~MyDecomposition() override = default;

        void project(const ob::State* s, std::vector<double>& coord) const override
            {
            coord.resize(2);
            coord[0] = s->as<ob::RealVectorStateSpace::StateType>()->values[0];
            coord[1] = s->as<ob::RealVectorStateSpace::StateType>()->values[1];
            }

        void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
            {
            sampler->sampleUniform(s);
            s->as<ob::RealVectorStateSpace::StateType>()->values[0] = coord[0];
            s->as<ob::RealVectorStateSpace::StateType>()->values[1] = coord[1];
            }
    };

void loadEnv(std::string fname, std::vector<Polygon>& obstacles, std::vector<Polygon>& regions)
    {
    std::ifstream envFile;
    envFile.open("../envs/" + fname);
    JSON j;
    if (envFile.is_open())
        {
        envFile >> j;
        }
    else
        {
        throw std::runtime_error("Unable to open environment file");
        }
    for (auto& e : j["obstacles"])
        {
        size_t n = e["pts"].size();
        std::cout << "Loading obstacle with " << n << " points" << std::endl;
        Polygon poly(n);
        for (int i = 0; i < n; ++i)
            {
            float x = e["pts"][i][0];
            float y = e["pts"][i][1];
            poly.pts[i] = Vertex(x, y);
            }
        obstacles.push_back(poly);
        }
    for (auto& e : j["regions"])
        {
        size_t n = e["pts"].size();
        Polygon poly(n);
        for (int i = 0; i < n; ++i)
            {
            float x = e["pts"][i][0];
            float y = e["pts"][i][1];
            poly.pts[i] = Vertex(x, y);
            }
        regions.push_back(poly);
        }
    std::cout << "Loaded " << obstacles.size() << " obstacles and " << regions.size() << " regions" << std::endl;
    }

void addObstaclesAndPropositions(std::shared_ptr<oc::PropositionalTriangularDecomposition>& decomp, std::vector<Polygon>& obstacles, std::vector<Polygon>& regions)
    {
    for (auto o : obstacles)
        {
        decomp->addHole(o);
        }

    for (auto r : regions)
        {
        decomp->addProposition(r);
        }
    }

/* Returns whether a point (x,y) is within a given convex polygon. */
bool polyContains(const Polygon& poly, double x, double y)
    {
    // Adapted from https://stackoverflow.com/questions/1119627/how-to-test-if-a-point-is-inside-of-a-convex-polygon-in-2d-integer-coordinates
    const size_t n = poly.pts.size();
    assert(n > 2); // Ensure that we are testing a polygon and not a line or point

    unsigned int pos = 0;
    unsigned int neg = 0;

    for (int i = 0; i < n; i++)
        {
        float x1 = poly.pts[i].x;
        float y1 = poly.pts[i].y;
        // Test if test point is a vertex of the polygon
        if ((x1 == x) && (y1 == y))
            {
            return true;
            }

        //And the i+1'th, or if i is the last, with the first point
        float i2 = (i + 1) % n;

        float x2 = poly.pts[i2].x;
        float y2 = poly.pts[i2].y;

        //Compute the cross product
        float d = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1);

        if (d > 0)
            {
            pos++;
            }
        if (d < 0)
            {
            neg++;
            }

        //If the sign changes, then point is outside
        if (pos > 0 && neg > 0)
            {
            return false;
            }
        }
    //If no change in direction, then on same side of all segments, and thus inside
    return true;
    }

/* Our state validity checker queries the decomposition for its obstacles,
   and checks for collisions against them.
   This is to prevent us from having to redefine the obstacles in multiple places. */
bool isStateValid(const oc::SpaceInformation* si, const std::shared_ptr<oc::PropositionalTriangularDecomposition>& decomp, const ob::State* state)
    {
    if (!si->satisfiesBounds(state))
        {
        return false;
        }

    const double x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    const double y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
    const std::vector<Polygon>& obstacles = decomp->getHoles();
    for (const auto& obstacle : obstacles)
        {
        if (polyContains(obstacle, x, y))
            return false;
        }
    return true;
    }

void propagate(const ob::State* start, const oc::Control* control, const double duration, ob::State* result)
    {
    const auto* ctrl = control->as<oc::RealVectorControlSpace::ControlType>();
    const auto* s = start->as<ob::RealVectorStateSpace::StateType>();

    // Split out the state and control variables
    const double x = s->values[0];
    const double y = s->values[1];
    const double vx = s->values[2];
    const double vy = s->values[3];
    const double u1 = ctrl->values[0];
    const double u2 = ctrl->values[1];
    const double t = duration;

    // Solved ODE Equations
    double xout = x + vx * t + (u1 * t * t) / 2;
    double yout = y + vy * t + (u2 * t * t) / 2;
    double vxout = vx + u1 * t;
    double vyout = vy + u2 * t;

    // Save the results
    result->as<ob::RealVectorStateSpace::StateType>()->values[0] = xout;
    result->as<ob::RealVectorStateSpace::StateType>()->values[1] = yout;
    result->as<ob::RealVectorStateSpace::StateType>()->values[2] = vxout;
    result->as<ob::RealVectorStateSpace::StateType>()->values[3] = vyout;
    }

void plan(JSON config)
    {
    std::ofstream pathFile;
    std::ofstream decompFile;
    pathFile.open("../sols/" + config["sol"].get<std::string>());
    decompFile.open("../envs/" + config["decomp"].get<std::string>());
    if (!pathFile.is_open() || !decompFile.is_open())
        {
        throw std::runtime_error("Unable to open solution or decomposition file");
        }

    // Load the environment
    std::vector<Polygon> obstacles;
    std::vector<Polygon> regions;
    loadEnv(config["env"].get<std::string>(), obstacles, regions);

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(4));

    // set the bounds for the state space
    ob::RealVectorBounds bounds(4);
    // Lower Bounds
    bounds.setLow(0, config["xbounds"][0]);  // x 
    bounds.setLow(1, config["ybounds"][0]);  // y
    bounds.setLow(2, -1); // vx
    bounds.setLow(3, -1); // vy
    // Upper Bounds
    bounds.setHigh(0, config["xbounds"][1]); // x
    bounds.setHigh(1, config["ybounds"][1]); // y
    bounds.setHigh(2, 1); // vx
    bounds.setHigh(3, 1); // vy

    space->setBounds(bounds);

    // Copy over the R^2 bounds for decomposition
    ob::RealVectorBounds decompBounds(2);
    decompBounds.setLow(0, bounds.low[0]);
    decompBounds.setLow(1, bounds.low[1]);
    decompBounds.setHigh(0, bounds.high[0]);
    decompBounds.setHigh(1, bounds.high[1]);

    // create triangulation that ignores obstacle and respects propositions
    std::shared_ptr<oc::PropositionalTriangularDecomposition> ptd = std::make_shared<MyDecomposition>(decompBounds);
    // helper method that adds an obstacle, as well as three propositions p0,p1,p2
    addObstaclesAndPropositions(ptd, obstacles, regions);
    ptd->setup();

    // Save the decomposition
    ptd->print(decompFile);
    printf("Wrote decomposition to file\n");

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-.5);
    cbounds.setHigh(.5);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));
    si->setStateValidityChecker([&si, ptd](const ob::State* state)
        {
        return isStateValid(si.get(), ptd, state);
        });
    si->setStatePropagator(propagate);
    si->setPropagationStepSize(config["step_size"]); //This is causing seg faults sometimes?

    //LTL co-safety sequencing formula: visit p2,p0 in that order
    auto cosafety = std::make_shared<oc::Automaton>(config["n_propositions"], config["cosafety"].get<std::string>());
    // TODO: Add option to turn off safety
    auto safety = std::make_shared<oc::Automaton>(config["n_propositions"], config["safety"].get<std::string>(), false);

    // TODO: Save out resulting automaton graphs
    // construct product graph (propDecomp x A_{cosafety} x A_{safety})
    auto product(std::make_shared<oc::ProductGraph>(ptd, cosafety, safety));

    // LTLSpaceInformation creates a hybrid space of robot state space x product graph.
    // It takes the validity checker from SpaceInformation and expands it to one that also
    // rejects any hybrid state containing rejecting automaton states.
    // It takes the state propagator from SpaceInformation and expands it to one that
    // follows continuous propagation with setting the next decomposition region
    // and automaton states accordingly.
    //
    // The robot state space, given by SpaceInformation, is referred to as the "lower space".
    auto ltlsi(std::make_shared<oc::LTLSpaceInformation>(si, product));

    // LTLProblemDefinition creates a goal in hybrid space, corresponding to any
    // state in which both automata are accepting
    auto pdef(std::make_shared<oc::LTLProblemDefinition>(ltlsi));

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = config["start"][0];
    start[1] = config["start"][1];
    start[2] = 0;
    start[3] = 0;

    // addLowerStartState accepts a state in lower space, expands it to its
    // corresponding hybrid state (decomposition region containing the state, and
    // starting states in both automata), and adds that as an official start state.
    pdef->addLowerStartState(start.get());

    //LTL planner (input: LTL space information, product automaton)
    oc::LTLPlanner ltlPlanner(ltlsi, product);
    ltlPlanner.setProblemDefinition(pdef);

    // attempt to solve the problem within thirty seconds of planning time
    // considering the above cosafety/safety automata, a solution path is any
    // path that visits p2 followed by p0 while avoiding obstacles and avoiding p1.
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(config["solve_time"]);
    // This can be used to reset the planner
    // ltlPlanner.ob::Planner::clear();
    // solved = ltlPlanner.ob::Planner::solve(30.0);

    if (solved)
        {
        std::cout << "Found solution, wrote to file" << std::endl;
        // The path returned by LTLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to LTLSpaceInformation.
        auto path = static_cast<oc::PathControl&>(*pdef->getLowerSolutionPath());
        path.printAsMatrix(pathFile);
        }
    else
        std::cout << "No solution found" << std::endl;
    }

int main(int argc, char** argv)
    {
    // TODO: Create configuration file that stores environment path, LTL formula, solution path. Use this as the sole parameter to pass
    if (argc == 2)
        {
        std::ifstream configFile;
        // First argument is environment file
        configFile.open(argv[1]);
        // outfile.open("test.txt");
        if (configFile.is_open())
            {
            JSON jConfigFile;
            configFile >> jConfigFile;
            configFile.close();

            plan(jConfigFile);
            }
        else
            {
            std::cout << "Failed to open configuration" << std::endl;
            }
        }
    else
        {
        std::cout << "Need to pass environment and output filenames" << std::endl;
        }
    // plan();
    return 0;
    }