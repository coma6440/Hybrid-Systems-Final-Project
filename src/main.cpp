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

/* Additional Libraries */
#include <json.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;

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
            // coord[0] = s->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getX();
            // coord[1] = s->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getY();
            // coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
            // coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
            coord[0] = s->as<ob::RealVectorStateSpace::StateType>()->values[0];
            coord[1] = s->as<ob::RealVectorStateSpace::StateType>()->values[1];
            }

        void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
            {
            sampler->sampleUniform(s);
            // auto* ws = s->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
            // ws->setXY(coord[0], coord[1]);
            s->as<ob::RealVectorStateSpace::StateType>()->values[0] = coord[0];
            s->as<ob::RealVectorStateSpace::StateType>()->values[1] = coord[1];
            }
    };

void loadEnv(JSON j, std::vector<Polygon>& obstacles, std::vector<Polygon>& regions)
    {
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

/* Returns whether a point (x,y) is within a given polygon.
   We are assuming that the polygon is a axis-aligned rectangle, with vertices stored
   in counter-clockwise order, beginning with the bottom-left vertex. */
bool polyContains(const Polygon& poly, double x, double y)
    {
    // TODO: Adapt this to be convex polygons
    return x >= poly.pts[0].x && x <= poly.pts[2].x
        && y >= poly.pts[0].y && y <= poly.pts[2].y;
    }

/* Our state validity checker queries the decomposition for its obstacles,
   and checks for collisions against them.
   This is to prevent us from having to redefine the obstacles in multiple places. */
bool isStateValid(
    const oc::SpaceInformation* si,
    const std::shared_ptr<oc::PropositionalTriangularDecomposition>& decomp,
    const ob::State* state)
    {
    if (!si->satisfiesBounds(state))
        return false;
    // const auto* se2 = state->as<ob::SE2StateSpace::StateType>();
    // const auto* se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);

    double x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    double y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
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
    // TODO: Adapt this for custom dynamics
    // const auto* se2 = start->as<ob::SE2StateSpace::StateType>();
    // const auto* se2 = start->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
    // const auto* vel = start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const auto* rctrl = control->as<oc::RealVectorControlSpace::ControlType>();
    const auto* s = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(start);
    double x = (*s)[0];
    double y = (*s)[1];
    double vx = (*s)[2];
    double vy = (*s)[3];
    double u1 = rctrl->values[0];
    double u2 = rctrl->values[1];
    double t = duration;

    // double xout = se2->getX() + rctrl->values[0] * duration * cos(se2->getYaw());
    // double yout = se2->getY() + rctrl->values[0] * duration * sin(se2->getYaw());
    // double yawout = se2->getYaw() + rctrl->values[1];
    double xout = x + vx * t + (u1 * t * t) / 2;
    double yout = y + vy * t + (u2 * t * t) / 2;
    double vxout = vx + u1 * t;
    double vyout = vy + u2 * t;

    result->as<ob::RealVectorStateSpace::StateType>()->values[0] = xout;
    result->as<ob::RealVectorStateSpace::StateType>()->values[1] = yout;
    result->as<ob::RealVectorStateSpace::StateType>()->values[2] = vxout;
    result->as<ob::RealVectorStateSpace::StateType>()->values[3] = vyout;
    // auto* se2out = result->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
    // se2out->setXY(xout, yout);
    // // se2out->setYaw(yawout);

    // auto* so2out = se2out->as<ob::SO2StateSpace::StateType>(1);
    // ob::SO2StateSpace SO2;
    // SO2.enforceBounds(so2out);
    }

void plan(JSON env_json)
    {
    // Load the environment
    std::vector<Polygon> obstacles;
    std::vector<Polygon> regions;
    loadEnv(env_json, obstacles, regions);
    // construct the state space we are planning in
    // auto space(std::make_shared<ob::SE2StateSpace>());

    // auto se2(std::make_shared<ob::SE2StateSpace>());
    // auto r2(std::make_shared<ob::RealVectorStateSpace>(2));
    auto space(std::make_shared<ob::RealVectorStateSpace>(4));

    // set the bounds for the R^2 part of SE(2)
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(0);
    // bounds.setHigh(2);
    // // set the bounds for the velocity portion of the state space
    // ob::RealVectorBounds velBounds(2);
    // velBounds.setLow(-1);
    // velBounds.setHigh(1);

    // se2->setBounds(bounds);
    // r2->setBounds(velBounds);

    ob::RealVectorBounds bounds(4);
    bounds.setLow(0, 0);
    bounds.setLow(1, 0);
    bounds.setLow(2, -1);
    bounds.setLow(3, -1);
    bounds.setHigh(0, 2);
    bounds.setHigh(1, 2);
    bounds.setHigh(2, 1);
    bounds.setHigh(3, 1);

    space->setBounds(bounds);

    // auto space = se2 + r2;

    // create triangulation that ignores obstacle and respects propositions
    std::shared_ptr<oc::PropositionalTriangularDecomposition> ptd = std::make_shared<MyDecomposition>(bounds);
    // helper method that adds an obstacle, as well as three propositions p0,p1,p2
    addObstaclesAndPropositions(ptd, obstacles, regions);
    ptd->setup();

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-.5);
    cbounds.setHigh(.5);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));
    si->setStateValidityChecker(
        [&si, ptd](const ob::State* state)
        {
        return isStateValid(si.get(), ptd, state);
        });
    si->setStatePropagator(propagate);
    si->setPropagationStepSize(0.025);

    //LTL co-safety sequencing formula: visit p2,p0 in that order
    // TODO: Try to install OMPL with SPOT
#if OMPL_HAVE_SPOT
    // This shows off the capability to construct an automaton from LTL-cosafe formula using Spot
    auto cosafety = std::make_shared<oc::Automaton>(3, "! p0 U ((p2 & !p0) & XF p0)");
#else
    auto cosafety = oc::Automaton::SequenceAutomaton(3, { 2, 0 });
#endif
    //LTL safety avoidance formula: never visit p1
#if OMPL_HAVE_SPOT
    // This shows off the capability to construct an automaton from LTL-safe formula using Spot
    auto safety = std::make_shared<oc::Automaton>(3, "G ! p1", false);
#else
    auto safety = oc::Automaton::AvoidanceAutomaton(3, { 1 });
#endif

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
    // ob::ScopedState<ob::SE2StateSpace> start(space);
    // start->setX(0.2);
    // start->setY(0.2);
    // start->setYaw(0.0);
    ob::ScopedState<> start(space);
    // start->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->setXY(0.2, 0.2);
    // start->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->setYaw(0.0);
    start[0] = 0.2;
    start[1] = 0.2;

    std::cout << start;

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
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(30.0);

    if (solved)
        {
        std::cout << "Found solution:" << std::endl;
        // The path returned by LTLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to LTLSpaceInformation.
        // TODO: Save to separate output file passed into function
        static_cast<oc::PathControl&>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);
        }
    else
        std::cout << "No solution found" << std::endl;
    }

int main(int argc, char** argv)
    {
    if (argc == 2)
        {
        std::ifstream file;
        // First argument is environment file
        file.open(argv[1]);
        if (file.is_open())
            {
            JSON j;
            file >> j;
            file.close();
            std::cout << "Opened environment: " << j["env_name"] << std::endl;
            plan(j);
            }
        }
    else
        {
        std::cout << "Need to pass environment file" << std::endl;
        }
    // plan();
    return 0;
    }