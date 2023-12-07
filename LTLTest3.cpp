/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
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
  
 /* Author: Matt Maly */
#include "src/includeCore.h"
#include "src/MyGridDecomposition.h"
#include "src/MyPropDecomposition.h"
#include <fstream>
  
 namespace ob = ompl::base;
 namespace oc = ompl::control;
  
 using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
 using Vertex = oc::PropositionalTriangularDecomposition::Vertex;
  
 // a decomposition is only needed for SyclopRRT and SyclopEST
 // use TriangularDecomp
  
 void addPropositions(std::shared_ptr<MyPropDecomposition> &decomp)
 {
    //Specify the region for each proposition
    int p0 = 3;
    decomp->addProposition(p0);

    int p1 = 14;
    decomp->addProposition(p1);

    int p2 = 21;
    decomp->addProposition(p2);

    int p3 = 7;
    decomp->addProposition(p3);

    int p4 = 9;
    decomp->addProposition(p4);
 }
  
 /* Returns whether a point (x,y) is within a given polygon.
    We are assuming that the polygon is a axis-aligned rectangle, with vertices stored
    in counter-clockwise order, beginning with the bottom-left vertex. */
//  bool polyContains(const Polygon& poly, double x, double y)
//  {
//      return x >= poly.pts[0].x && x <= poly.pts[2].x
//          && y >= poly.pts[0].y && y <= poly.pts[2].y;
//  }
  
 /* Our state validity checker queries the decomposition for its obstacles,
    and checks for collisions against them.
    This is to prevent us from having to redefine the obstacles in multiple places. */
 bool isStateValid(
    const oc::SpaceInformation *si,
    const std::shared_ptr<oc::PropositionalDecomposition> &decomp,
    const ob::State *state)
 {
    if (!si->satisfiesBounds(state)){
        //std::cout << "State does not satisfy bounds" << std::endl;
        return false;
    }
    //  const auto* se2 = state->as<ob::SE2StateSpace::StateType>();
  
    //  double x = se2->getX();
    //  double y = se2->getY();
    //  const std::vector<Polygon>& obstacles = decomp->getHoles();
    //  for (const auto & obstacle : obstacles)
    //  {
    //      if (polyContains(obstacle, x, y))
    //          return false;
    //  }
    return true;
 }
  
 void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
 {
     const auto* se2 = start->as<ob::SE2StateSpace::StateType>();
     const auto* rctrl = control->as<oc::RealVectorControlSpace::ControlType>();
  
     double xout = se2->getX() + rctrl->values[0]*duration*cos(se2->getYaw());
     double yout = se2->getY() + rctrl->values[0]*duration*sin(se2->getYaw());
     double yawout = se2->getYaw() + rctrl->values[1];
  
     auto* se2out = result->as<ob::SE2StateSpace::StateType>();
     se2out->setXY(xout, yout);
     se2out->setYaw(yawout);
  
     auto* so2out = se2out->as<ob::SO2StateSpace::StateType>(1);
     ob::SO2StateSpace SO2;
     SO2.enforceBounds (so2out);
 }

 void printProblem(std::shared_ptr<MyPropDecomposition> ptd, int len, ob::RealVectorBounds bounds){
        //Open the file to save the problem stats
        std::ofstream myfile;

        //Open the file
        myfile.open("problem.txt");

        //Write the problem stats to the file
        myfile << "dimensions, " << ptd->getDimension() << std::endl; 
        myfile << "length, " << len << std::endl;
        myfile << "bounds_low, " << bounds.low[0] << std::endl;
        myfile << "bounds_high, " << bounds.high[0] << std::endl;
        //Add the propositions
        myfile << "propositions, " << ptd->getNumProps() << std::endl;
        for (int i = 0; i < ptd->getNumProps(); i++){
            myfile << "proposition_" << i << ", " << ptd->getProposition(i) << std::endl;
        }
        
        //Close the file
        myfile.close();
 }
  
 void plan()
 {
    // Grid Space Parameters
    int length = 5; // Number of grid cells along each axis
    int dim = 2; // Number of dimensions

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(dim);
    bounds.setLow(0);
    bounds.setHigh(2);

    space->setBounds(bounds);
  
    //Instantiate the grid decomposition
    std::shared_ptr<oc::Decomposition> grid = std::make_shared<MyGridDecomposition>(length, dim, bounds);
         
    //Print the grid decomposition variables to the console
    MyGridDecomposition grid_test(length, dim, bounds);
    grid_test.print();

    //Pass this grid decomposition to the propositional decomposition
    std::shared_ptr<MyPropDecomposition> ptd = std::make_shared<MyPropDecomposition>(grid);

    // Add the regions of interest to the propositional decomposition
    addPropositions(ptd);
    //ptd->setup();
   
    //Print the problem
    printProblem(ptd, length, bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, dim));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(dim);
    cbounds.setLow(-2);
    cbounds.setHigh(2);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));

    si->setStateValidityChecker(
        [&si, ptd](const ob::State *state)
        {
            return isStateValid(si.get(), ptd, state);
        });

    si->setStatePropagator(propagate);
    si->setPropagationStepSize(0.025);
  
              
     //LTL co-safety sequencing formula: visit p2, p1 in that order
 #if OMPL_HAVE_SPOT
     // This shows off the capability to construct an automaton from LTL-cosafe formula using Spot
     auto cosafety = std::make_shared<oc::Automaton>(3, "! p0 U ((p2 & !p0) & XF p0)");
 #else
     auto cosafety = oc::Automaton::SequenceAutomaton(3, {0, 1, 2});
 #endif
     //LTL safety avoidance formula: Just visit p1
 #if OMPL_HAVE_SPOT
     // This shows off the capability to construct an automaton from LTL-safe formula using Spot
     auto safety = std::make_shared<oc::Automaton>(3, "G ! p1", false);
 #else
     //auto safety = oc::Automaton::AvoidanceAutomaton(3, {0});
     auto safety = oc::Automaton::AvoidanceAutomaton(3, {3, 4});
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
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.2);
    start->setY(0.2);
    start->setYaw(0.0);

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
    ltlPlanner.printProperties(std::cout);
    ltlPlanner.printSettings(std::cout);
    ltlPlanner.checkValidity();
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(10.0);

     //DEBUG planner
    //std::vector<ob::State *> tree;
    //ltlPlanner.getTree(tree);
    //std::cout << "Tree size: " << tree.size() << std::endl;
    // for (int i = 0; i < 10; i++){
    //     //Get the X and Y coordinates of each state in the tree
    //     const auto* se2 = tree[i]->as<ob::SE2StateSpace::StateType>();
    //     double x = se2->getX();
    //     double y = se2->getY();
    //     std::cout << "State " << i << " X: " << x << " Y: " << y << std::endl;
    // }

    //Get the high level path of the planner:
    
  
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // The path returned by LTLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to LTLSpaceInformation.
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);

        //Open a text file to write the path to
        std::ofstream myfile;

        //Open the file
        myfile.open("path.txt");

        //Write the path to the file
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(myfile);

        //Close the file
        myfile.close();

    }
    else
        std::cout << "No solution found" << std::endl;
        //static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);
    
 }
  
 int main(int /*argc*/, char ** /*argv*/)
 {
     plan();
     return 0;
 }