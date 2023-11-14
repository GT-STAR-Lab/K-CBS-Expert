/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include "Robot.h"
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "PlannerAllocatorDatabase.h"

#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <unordered_map>
#include <math.h>
#include <chrono>
#include "rapidxml/rapidxml.hpp"

using namespace rapidxml;
namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

void plan(int width, int height, std::unordered_map<std::string, std::pair<int, int>> start_map, std::unordered_map<std::string, std::pair<int, int>> goal_map, char* planFileName)
{
    // construct all of the robots (assume square robots with unit length)
    std::unordered_map<std::string, Robot*> robot_map;
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++)
    {
        Robot* robot = new RectangularRobot(itr->first, 1.0, 0.5);
        robot_map[itr->first] = robot;
    }

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrc::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

    // construct four individuals that operate in SE3
    for (auto itr = start_map.begin(); itr != start_map.end(); itr++) 
    {
        // construct the state space we are planning in
        auto space = createBounded2ndOrderCarStateSpace(width, height);

        // name the state space parameter
        space->setName(itr->first);

        // create a control space
        auto cspace = createUniform2DRealVectorControlSpace(space);

        // construct an instance of  space information from this control space
        auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

        // // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<homogeneous2ndOrderCarSystemSVC>(si, robot_map));

        // set the state propagation routine
        auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &KinematicBicycleODE));
        si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderCarODEPostIntegration));

        // set the propagation step size
        si->setPropagationStepSize(1);

        // set this to remove the warning
        si->setMinMaxControlDuration(1, 10);

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = start_map.at(itr->first).first;
        start[1] = start_map.at(itr->first).second;
        start[2] = 0.0;
        start[3] = 0.0;
        start[4] = 0.0;

        // // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
        pdef->addStartState(start);
        pdef->setGoal(std::make_shared<GoalRegion2ndOrderCar>(si, goal_map.at(itr->first)));

        // add the individual information to the multi-robot SpaceInformation and ProblemDefinition
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);
    }
    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();

    // set the planner allocator for the multi-agent planner
    ompl::base::PlannerAllocator allocator = &allocateControlRRT;
    ma_si->setPlannerAllocator(allocator);

    // plan with K-CBS
    auto planner = std::make_shared<omrc::KCBS>(ma_si);
    planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition
    planner->setLowLevelSolveTime(5.);

    auto start = std::chrono::high_resolution_clock::now();
    bool solved = planner->as<omrb::Planner>()->solve(180.0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    double duration_s = (duration_ms.count() * 0.001);

    if (solved)
    {
        printf("Found Solution in %0.2f seconds!\n", duration_s);
       
        omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
        std::this_thread::sleep_for (std::chrono::milliseconds(1)); //sometimes segfaults without this
        std::ofstream MyFile(planFileName);
        
        omrc::PlanControl *planControl = solution->as<omrc::PlanControl>();
        if(planControl)
            planControl->printAsMatrix(MyFile, "Robot");
        else
            std::cout << "FAILED TO WRITE TO FILE"<<std::endl;
        MyFile.close();        
    }
    std::ofstream MyFile2("tree.txt");
    planner->printConstraintTree(MyFile2);
    MyFile2.close();
}

int main(int argc, char ** argv)
{
    if (argc != 3) {
        std::cout << "Usage: ./program_name <xml_file> <out_path_file>\n";
        return 1;
    }

    const char* xmlFileName = argv[1];

    // Define variables for width, height, and unordered maps to store start and goal locations
    int width = 0, height = 0;
    std::unordered_map<std::string, std::pair<int, int>> startLocations;
    std::unordered_map<std::string, std::pair<int, int>> goalLocations;

    // Read the XML file
    std::ifstream file(xmlFileName);
    if (!file.is_open()) {
        std::cout << "Unable to open file: " << xmlFileName << "\n";
        return 1;
    }

    // Read the content of the file into a string
    std::string xmlContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // Parse the XML content
    xml_document<> doc;
    doc.parse<0>(&xmlContent[0]);

    // Get the root node
    xml_node<>* rootNode = doc.first_node("root");
    if (rootNode == nullptr) {
        std::cout << "Invalid XML format: Missing root node 'root'\n";
        return 1;
    }

    // Extract width and height values
    xml_attribute<>* widthAttr = rootNode->first_attribute("width");
    xml_attribute<>* heightAttr = rootNode->first_attribute("height");

    if (widthAttr && heightAttr) {
        width = std::stoi(widthAttr->value());
        height = std::stoi(heightAttr->value());
    } else {
        std::cout << "Missing width or height attribute\n";
        return 1;
    }

    // Iterate through each agent node
    for (xml_node<>* agentNode = rootNode->first_node("agent"); agentNode; agentNode = agentNode->next_sibling("agent")) {
        xml_attribute<>* startIAttr = agentNode->first_attribute("start_i");
        xml_attribute<>* startJAttr = agentNode->first_attribute("start_j");
        xml_attribute<>* goalIAttr = agentNode->first_attribute("goal_i");
        xml_attribute<>* goalJAttr = agentNode->first_attribute("goal_j");

        if (startIAttr && startJAttr && goalIAttr && goalJAttr) {
            // Store start and goal locations in unordered maps
            std::string agentName = "Robot " + std::to_string(startLocations.size() + 1);
            startLocations[agentName] = std::make_pair(std::stoi(startIAttr->value()), std::stoi(startJAttr->value()));
            goalLocations[agentName] = std::make_pair(std::stoi(goalIAttr->value()), std::stoi(goalJAttr->value()));
        }
    }

    std::cout << "Planning for 10 2nd order cars inside an Empty "<< width << "x" << height <<" workspace with K-CBS." << std::endl;
    plan(width, height, startLocations, goalLocations, argv[2]);
}
