 #pragma once
 
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

//Include opencv
 #include <opencv2/opencv.hpp>
 