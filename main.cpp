
#include "src/includeCore.h"
#include "src/MyGridDecomposition.h"
#include "src/MyPropDecomposition.h"
#include "src/helperFunctions.h"
#include <iostream>
#include <fstream>
#include <vector>
  
 namespace ob = ompl::base;
 namespace oc = ompl::control;
  
 using Polygon = oc::PropositionalTriangularDecomposition::Polygon;
 using Vertex = oc::PropositionalTriangularDecomposition::Vertex;
  

  
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
    const std::shared_ptr<MyPropDecomposition> &decomp,
    const ob::State *state)
{
    if (!si->satisfiesBounds(state)){
        //std::cout << "State does not satisfy bounds" << std::endl;
        return false;
    }
    const auto* se2 = state->as<ob::SE2StateSpace::StateType>();

    double x = se2->getX();
    double y = se2->getY();

    //Check if the state is in an obstacle
    int rid = decomp->locateRegion(se2);
    if (decomp->regionStatus(rid) == 0){
        //std::cout << "State is in an obstacle" << std::endl;
        return false;
    }

    //Check if the state neighbors are obstacles
    std::vector<int> neighbors;
    decomp->getNeighbors(rid, neighbors);
    for (int i = 0; i < neighbors.size(); i++){
        if (decomp->regionStatus(neighbors[i]) == 0){
            //std::cout << "State is in an obstacle" << std::endl;
            return false;
        }
    }

    return true ;
}
  
 void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
 {
     const auto* se2 = start->as<ob::SE2StateSpace::StateType>();
     const auto* rctrl = control->as<oc::RealVectorControlSpace::ControlType>();
  
     double yawout = se2->getYaw() + ((rand()%2)-1)*(3.1415/32);
     double xout = se2->getX() + rctrl->values[0]*duration*cos(se2->getYaw());
     double yout = se2->getY() + rctrl->values[0]*duration*sin(se2->getYaw());
    
  
     auto* se2out = result->as<ob::SE2StateSpace::StateType>();
     se2out->setXY(xout, yout);
     se2out->setYaw(yawout);

    //Print the state to the console
    //std::cout << "X: " << xout << " Y: " << yout << " Yaw: " << yawout << std::endl;

    //Print the control to the console
    //std::cout << "Control: " << rctrl->values[0] << " " << rctrl->values[1] << std::endl;
  
     auto* so2out = se2out->as<ob::SO2StateSpace::StateType>(1);
     ob::SO2StateSpace SO2;
     SO2.enforceBounds (so2out);
 }
  
 void plan()
 {
    // Grid Space Parameters
    int length = 20; // Number of grid cells along each axis
    int dim = 2; // Number of dimensions
    unsigned int propCount = 4; // Number of propositions
    unsigned int safetyCount = 1; // Number of safety propositions
    unsigned int obstacleCount = 10; //(length*length)/10; // Number of obstacles
    double bound_max = 1; // Maximum bound of the grid
    double stepSize = 0.04; // Step size for the planner

    bool land = true; // Whether or not the drone returns to home base
    bool useImageMap = true; // Whether or not to use an image map
    bool useRandomProps = true; // Whether or not to use random propositions

    std::string image_path = "/home/mini/Drone_LTL_Planner/maps/Forest_Map_Hard.jpg"; // Path to the image map
    cv::Mat image; // Image map 

    //If we are using an image map, use the image map
    if (useImageMap){
        // Open up the image specified by image_path
        image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

        // Check for invalid input
        if(! image.data )
        {
            std::cout <<  "Could not open or find the image" << std::endl ;
            return;
        }

        // Set the dimension as 2
        dim = 2;

        // Set the length as the pixel width of the image
        length = image.cols;
    }

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(dim);
    bounds.setLow(0);
    bounds.setHigh(bound_max);

    space->setBounds(bounds);
  
    //Instantiate the grid decomposition
    std::shared_ptr<oc::Decomposition> grid = std::make_shared<MyGridDecomposition>(length, dim, bounds);
         
    //Print the grid decomposition variables to the console
    MyGridDecomposition grid_test(length, dim, bounds);
    grid_test.print();

    //Pass this grid decomposition to the propositional decomposition
    std::shared_ptr<MyPropDecomposition> ptd = std::make_shared<MyPropDecomposition>(grid);

    //If an image was use, set any pixel that is not white as an obstacle
    if (useImageMap){
        addImageObstacles(ptd, image, space);

        //Add the regions of interest to the propositional decomposition
        if (useRandomProps){
            addPropositions(ptd, propCount, safetyCount, land);
        } else{
            specificProps(ptd, propCount, land);
            specificObstacles(ptd, obstacleCount);
        }

    } else{

        // Add the regions of interest to the propositional decomposition
        addPropositions(ptd, propCount, safetyCount, land);
        addCircleObstacles(ptd, obstacleCount);
    }
   
    //Print the problem
    printProblem(ptd, length, bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, dim));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(dim);
    cbounds.setLow(-1);
    cbounds.setHigh(1);

    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(space, cspace));

    si->setStateValidityChecker(
        [&si, ptd](const ob::State *state)
        {
            return isStateValid(si.get(), ptd, state);
        });

    si->setStatePropagator(propagate);
    si->setPropagationStepSize(stepSize);

    // Add the landing proposition if the drone is returning to home base
    if (land){
        propCount++;
    }

    //Make vectors for the propositions to be followed
    std::vector<unsigned int> p_co;
    std::vector<unsigned int> p_safe;
    for (unsigned int i = 0; i < propCount; i++){
        p_co.push_back(i);
    }
    for (unsigned int i = propCount; i < propCount + safetyCount; i++){
        p_safe.push_back(i);
    }
  
              
     //LTL co-safety sequencing formula: visit p2, p1 in that order
 #if OMPL_HAVE_SPOT
     // This shows off the capability to construct an automaton from LTL-cosafe formula using Spot
     auto cosafety = std::make_shared<oc::Automaton>(3, "! p0 U ((p2 & !p0) & XF p0)");
 #else
    // Visit all the targets in order, +1 makes the drone go back to base
     auto cosafety = oc::Automaton::SequenceAutomaton(propCount, p_co);
 #endif
     //LTL safety avoidance formula: Just visit p1
 #if OMPL_HAVE_SPOT
     // This shows off the capability to construct an automaton from LTL-safe formula using Spot
     auto safety = std::make_shared<oc::Automaton>(3, "G ! p1", false);
 #else
     //auto safety = oc::Automaton::AvoidanceAutomaton(3, {0});
     auto safety = oc::Automaton::AvoidanceAutomaton(safetyCount, p_safe);
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
    start->setX(bound_max / (2*length));
    start->setY(bound_max / (2*length));
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
    ob::PlannerStatus solved = ltlPlanner.ob::Planner::solve(60.0);


    //Get the high level path of the planner:
    
  
    if (solved)
    {
        std::cout << "Found solution" << std::endl;
        // The path returned by LTLProblemDefinition is through hybrid space.
        // getLowerSolutionPath() projects it down into the original robot state space
        // that we handed to LTLSpaceInformation.
        //static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(std::cout);

        //Open a text file to write the path to
        std::ofstream myfile;

        //Open the file
        myfile.open("pathRaw.txt");

        //Write the path to the file
        static_cast<oc::PathControl &>(*pdef->getLowerSolutionPath()).printAsMatrix(myfile);
        
        //Write the start point to the file
        myfile << start->getX() << " " << start->getY() << " " << start->getYaw() << std::endl;

        //Close the file
        myfile.close();

        //Open up pathRaw.txt and save every tenth line to path.txt
        std::ifstream infile("pathRaw.txt");
        std::ofstream outfile("path.txt");
        std::string line;
        int lineCount = 0;
        while (std::getline(infile, line))
        {
            if (lineCount % 10 == 0){
                outfile << line << std::endl;
            }
            lineCount++;
        }
        outfile << line << std::endl;
        infile.close();
        outfile.close();

        //Post process the path to fit to the grid
        gridFit(ptd, bound_max, length, space);

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