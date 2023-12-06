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
  
 /* Author: Ioan Sucan */
  
 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/SE3StateSpace.h>
 #include <ompl/geometric/planners/rrt/RRTConnect.h>
 #include <ompl/geometric/SimpleSetup.h>
  
 #include <ompl/control/SpaceInformation.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/SimpleSetup.h>
 #include <ompl/config.h>
 #include <iostream>
 #include <vector>
  
 #include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
 #include <ompl/control/planners/ltl/PropositionalDecomposition.h>
 #include <ompl/control/planners/ltl/Automaton.h>
 #include <ompl/control/planners/ltl/ProductGraph.h>
 #include <ompl/control/planners/ltl/LTLPlanner.h>
 #include <ompl/control/planners/ltl/LTLProblemDefinition.h>
  
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
 namespace oc = ompl::control;

class MyDecomposition : public oc::PropositionalDecomposition
{
    public:
        MyDecomposition(oc::DecompositionPtr &decomp)
            : oc::PropositionalDecomposition(decomp) {}

        ~MyDecomposition() override = default;
  
        /** \brief Project a given State to a set of coordinates in R^k, where k is the dimension of this
             * Decomposition. */
        void project(const ob::State* s, std::vector<double>& coord) const override
        {
            coord.resize(2);
            coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
            coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
        }
    
        /** \brief Samples a State using a projected coordinate and a StateSampler. */
        void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
        {
            sampler->sampleUniform(s);
            auto* ws = s->as<ob::SE2StateSpace::StateType>();
            ws->setXY(coord[0], coord[1]);
        }
};
  
 bool isStateValid(const ob::State *state)
 {
     // cast the abstract state type to the type we expect
     const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
  
     // extract the first component of the state and cast it to what we expect
     const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  
     // extract the second component of the state and cast it to what we expect
     const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
  
     // check validity of state defined by pos & rot
  
  
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return (const void*)rot != (const void*)pos;
 }
  
 void plan()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE3StateSpace>());
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-1);
     bounds.setHigh(1);
  
     space->setBounds(bounds);
  
     // construct an instance of  space information from this state space
     auto si(std::make_shared<ob::SpaceInformation>(space));
  
     // set state validity checking for this space
     si->setStateValidityChecker(isStateValid);
  
     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();
  
     // create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  
     // set the start and goal states
     pdef->setStartAndGoalStates(start, goal);
  
     // create a planner for the defined space
     auto planner(std::make_shared<og::RRTConnect>(si));
  
     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);
  
     // perform setup steps for the planner
     planner->setup();
  
  
     // print the settings for this space
     si->printSettings(std::cout);
  
     // print the problem settings
     pdef->print(std::cout);
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
  
     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Found solution:" << std::endl;
  
         // print the path to screen
         path->print(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
 }
  
//  void planWithSimpleSetup()
//  {
//      // construct the state space we are planning in
//      auto space(std::make_shared<ob::SE2StateSpace>());
  
//      // set the bounds for the R^2 part of SE(2)
//      ob::RealVectorBounds bounds(2);
//      bounds.setLow(0);
//      bounds.setHigh(2);
  
//      space->setBounds(bounds);

//      //Create a basic LTL automaton. This is a sequence automaton with two states, and the transition is 1->0
//      auto automaton = ompl::control::Automaton::SequenceAutomaton(2, {1, 0});

//      // define a simple setup class
//      og::SimpleSetup ss(space);
  
//      // set state validity checking for this space
//      ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
  
//      // create a random start state
//      ob::ScopedState<> start(space);
//      start.random();
  
//      // create a random goal state
//      ob::ScopedState<> goal(space);
//      goal.random();
  
//      // set the start and goal states
//      ss.setStartAndGoalStates(start, goal);
  
//      // this call is optional, but we put it in to get more output information
//      //ss.setup();
//      //ss.print();
  
//      // attempt to solve the problem within one second of planning time
//      ob::PlannerStatus solved = ss.solve(1.0);
  
//      if (solved)
//      {
//          std::cout << "Found solution:" << std::endl;
//          // print the path to screen
//          ss.simplifySolution();
//          ss.getSolutionPath().print(std::cout);
//      }
//      else
//          std::cout << "No solution found" << std::endl;
//  }
  
 int main(int /*argc*/, char ** /*argv*/)
 {
     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
     //ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  
     plan();
  
     std::cout << std::endl << std::endl;
  
     //planWithSimpleSetup();
  
     return 0;
 }