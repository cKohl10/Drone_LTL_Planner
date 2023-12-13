#pragma once
#include "includeCore.h"
#include "MyGridDecomposition.h"
#include "MyPropDecomposition.h"

//Specific propositions for the forest map low
void specificProps(std::shared_ptr<MyPropDecomposition> &decomp, unsigned int &propCount, bool land){
    std::vector<int> props = {297, 308, 835, 782}; //{296, 835, 782};
    
    for (int i = 0; i < props.size(); i++){
        //Check that rid is a free space
        if (decomp->regionStatus(props[i]) == -1){
            decomp->addProposition(props[i], false);
            continue;
        }
        std::cout << "Proposition INVALID: " << props[i] << std::endl;
        
    }
    propCount = props.size();

    //Go back to the starting base region
    if (land){
        decomp->addProposition(0, false);
    }

    decomp->addProposition(427, true);

}

void specificObstacles(std::shared_ptr<MyPropDecomposition> &decomp, unsigned int &obstacleCount){
    std::vector<int> obstacles = {324, 226, 399, 243, 574, 706, 886, 802};

    for (int i = 0; i < obstacles.size(); i++){
        //Check that rid is a free space
        if (decomp->regionStatus(obstacles[i]) == -1 || decomp->regionStatus(obstacles[i]) == -2){
            decomp->addObstacles(obstacles[i]);
            continue;
        }
        std::cout << "Obstacle INVALID: " << obstacles[i] << std::endl;
        
    }
    obstacleCount = obstacles.size();
}

void addPropositions(std::shared_ptr<MyPropDecomposition> &decomp, int propCount, int safetyCount, bool land)
{
    std::cout << "Adding propositions..." << std::endl;

    //Random number generator
    // std::random_device rd;  // Non-deterministic random number generator
    // std::mt19937 gen(rd()); // Seed the generator
    // std::uniform_int_distribution<> distr(1, decomp->getNumRegions()); // Define the range

    //Specify the region for each proposition, adds an extra for safety
    for (int i = 0; i < propCount; i++){
        // Generate a random RID for the proposition
        int rid = rand() % decomp->getNumRegions();

        // Generate a random RID for the proposition until the region ID is a free space
        while(decomp->regionStatus(rid) != -1){
            rid = rand() % decomp->getNumRegions();
        }

        std::cout << "Proposition RID: " << rid << std::endl;

        decomp->addProposition(rid, false);
    }

    //Go back to the starting base region
    if (land){
        decomp->addProposition(0, false);
    }

    std::cout << "Adding safety..." << std::endl;
    //Specify the region for each proposition, adds an extra for safety
    for (int i = 0; i < safetyCount; i++){
        // Generate a random RID for the proposition
        int rid = rand() % decomp->getNumRegions();

        // Generate a random RID for the proposition until the region ID is a free space
        while(decomp->regionStatus(rid) != -1){
            rid = rand() % decomp->getNumRegions();
        }

        std::cout << "Proposition RID: " << rid << std::endl;

        decomp->addProposition(rid, true);
    }
}

void addObstacles(std::shared_ptr<MyPropDecomposition> &decomp, int obstacleCount)
{
    std::cout << "Adding obstacles..." << std::endl;
    //Specify the region for each obstacle
    for (int i = 0; i < obstacleCount; i++){
        // Generate a random RID for the obstacle until the region ID is a free space
        bool valid = true;
        int rid;
        while(!valid){
            // Generate a random RID for the obstacle
            rid = rand() % (rand() % (decomp->getNumRegions()-1)) + 1;

            //Check its neighbors for a proposition
            std::vector<int> neighbors;
            decomp->getNeighbors(rid, neighbors);
            for (int i = 0; i < neighbors.size(); i++){
                if (decomp->regionStatus(neighbors[i]) == 1){
                    valid = false;
                }
            }

            //Check to see if its a free space
            if (decomp->regionStatus(rid) != -1 || valid == false){
                continue;
            }
            
            //If it got here it passed all checks
            valid = true;
        }

        std::cout << "Obstacle RID: " << rid << std::endl;

        decomp->addObstacles(rid);
    }
}

void addCircleObstacles(std::shared_ptr<MyPropDecomposition> &decomp, int obstacleCount)
{
    std::cout << "Adding obstacles..." << std::endl;
    //Specify the region for each obstacle
    for (int i = 0; i < obstacleCount; i++){
        // Generate a random RID for the obstacle
        int rid = (rand() % (decomp->getNumRegions()-1)) + 1;

        // Generate a random RID for the obstacle until the region ID is a free space
        while(decomp->regionStatus(rid) == 1 || decomp->regionStatus(rid) == 2){
            rid = rand() % (rand() % (decomp->getNumRegions()-1)) + 1;
        }

        //Check to see if neighboring regions are free too
        std::vector<int> neighbors;
        decomp->getNeighbors(rid, neighbors);
        bool free = true;
        for (int i = 0; i < neighbors.size(); i++){
            if (decomp->regionStatus(rid) == 1 || decomp->regionStatus(rid) == 2){
                continue;
            }
            decomp->addObstacles(neighbors[i]);
        }

        std::cout << "Obstacle RID: " << rid << std::endl;

        decomp->addObstacles(rid);
    }
}

void addImageObstacles(std::shared_ptr<MyPropDecomposition> &decomp, cv::Mat image, ob::StateSpacePtr space){

    //Iterate through the image and find the obstacles
    for (int i = 0; i < image.rows; i++){
        for (int j = 0; j < image.cols; j++){
            //If the pixel is not white, add it to the list of obstacles
            if (image.at<uchar>(i, j) < 240){

                //Get the region ID of the point
                ob::ScopedState<ob::SE2StateSpace> state(space);

                //Convert pixel to state
                double x_coord = double(i)/image.cols;
                double y_coord = double(j)/image.rows;

                state->setX(x_coord);
                state->setY(y_coord);
                state->setYaw(0.0);

                //Get the region ID of the point
                int rid = decomp->locateRegion(state.get());

                //std::cout << "At pixel (" << j << ", " << i << ") Obstacle RID: " << rid << std::endl;

                //Check if the region is in the start state or near the start state
                if (rid == 0){
                    continue;
                }

                //Check to see if neighboring regions are free too
                std::vector<int> neighbors;
                decomp->getNeighbors(0, neighbors);
                bool free = true;
                for (int i = 0; i < neighbors.size(); i++){
                    if (rid == neighbors[i]){
                        free = false;
                    }
                }

                //If the region is free, add it to the list of obstacles
                if (free){
                    decomp->addObstacles(rid);
                }
            }
        }
    }
}

//This function will take in the path points and fit them to the grid
void gridFit(std::shared_ptr<MyPropDecomposition> &decomp, double bound_max, int length, ob::StateSpacePtr space){
    //Open the file to save the problem stats
    std::ofstream myfile;

    //Open the file
    myfile.open("pathGrid.txt");

    //Get the path from the file
    std::ifstream infile("pathRaw.txt");

    //Keep track of the previous region ID to avoid duplicates
    int prevRID = -1;
    std::vector<int> prevCoord = {0, 0};
    std::vector<int> prevDirection = {-10, 10};
    int repeat_count = 0; //Keep track of how many times the same direction is repeated

    //Get the first line of the file
    std::string line;
    while (std::getline(infile, line))
    {
        //Split the line into x, y, and yaw
        std::istringstream iss(line);
        std::string x, y, yaw;
        iss >> x >> y >> yaw;

        //Convert the string to a double
        double x_d = std::stod(x);
        double y_d = std::stod(y);
        double yaw_d = std::stod(yaw);

        //Convert the coordinates to a state
        ob::ScopedState<ob::SE2StateSpace> state(space);
        state->setX(x_d);
        state->setY(y_d);
        state->setYaw(yaw_d);

        //Get the region ID of the point
        int rid = decomp->locateRegion(state.get());

        //If the region ID is the same as the previous one, skip it
        if (rid == prevRID){
            continue;
        }

        //Convert the region id to coordinates
        std::vector<int> coord_c = decomp->getGridCoord(rid, length);
        double scaler = bound_max / length;

        //Get the direction of the point
        std::vector<int> direction = {coord_c[0] - prevCoord[0], coord_c[1] - prevCoord[1]};

        //If the direction is the same as the previous one, skip it
        if (direction == prevDirection){
            //Save the previous coordinates as the current ones
            prevCoord = coord_c;
            repeat_count++;
            continue;
        }

        if (repeat_count > 0){
            //Write the previous point to the file
            myfile << prevCoord[0] * scaler + scaler/2 << " " << prevCoord[1] * scaler + scaler/2 << " " << yaw_d << std::endl;
            repeat_count = 0;
        }


        //Write the center of the region to the file
        myfile << coord_c[0] * scaler + scaler/2 << " " << coord_c[1] * scaler + scaler/2 << " " << yaw_d << std::endl;

        //Save the previous coordinates as the current ones
        prevCoord = coord_c;

        //Set the previous region ID to the current one
        prevRID = rid;

        //Set the previous direction to the current one
        prevDirection = direction;
    }

    myfile << "0 0 0" << std::endl;

    //Close the file
    myfile.close();
    infile.close();

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
        myfile << "propositions, " << ptd->getNumCoSafeProps() << std::endl;
        for (int i = 0; i < ptd->getNumCoSafeProps(); i++){
            myfile << "proposition_" << i << ", " << ptd->getProposition(i) << std::endl;
        }
        //Add the obstacles
        myfile << "obstacles, " << ptd->getNumObstacles() << std::endl;
        for (int i = 0; i < ptd->getNumObstacles(); i++){
            myfile << "obstacle_" << i << ", " << ptd->getObstacle(i) << std::endl;
        }
        //Add proposition locations
        for (int i = 0; i < ptd->getNumCoSafeProps(); i++){
            std::vector<int> coords = ptd->getGridCoord(ptd->getProposition(i), len);
            double x = (double(coords[0]) * double(bounds.high[0] - bounds.low[0]) / double(len)) + double(bounds.low[0]);
            double y = (double(coords[1]) * double(bounds.high[0] - bounds.low[0]) / double(len)) + double(bounds.low[0]);

            //Save locations to file
            myfile << "proposition_location_" << i << ", " << x << ", " << y << std::endl;
        }
        
        //Close the file
        myfile.close();
 }
