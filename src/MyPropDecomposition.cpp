#include "MyPropDecomposition.h"

MyPropDecomposition::MyPropDecomposition(std::shared_ptr<oc::Decomposition> decomp)
    : oc::PropositionalDecomposition(decomp)
{
    // Constructor implementation...
    numCoSafeProps_ = 0;
    numSafeProps_ = 0;
}

oc::World MyPropDecomposition::worldAtRegion(int rid)
{
    int numProps = getNumProps();
    oc::World world(numProps);

    // Initialize all propositions to false
    for (int p = 0; p < numProps; ++p)
        world[p] = false;

    // If the region is -1, return the world with all propositions set to false
    if (rid == -1)
        return world;

    // Otherwise, set the proposition corresponding to the region to true
    int prop = -1;
    for (int i = 0; i < propRIDs_.size(); ++i)
    {
        if (propRIDs_[i] == rid)
        {
            prop = i;
            break;
        }
    }

    if (prop >= 0)
        world[prop] = true;


    return world;

}

int MyPropDecomposition::getNumProps() const
{
    return propRIDs_.size();
}   

int MyPropDecomposition::getNumCoSafeProps() const
{
    return numCoSafeProps_;
}  

std::vector<unsigned int> MyPropDecomposition::getPropositions(bool isSafety)
{
    if (!isSafety)
        return std::vector<unsigned int>(propRIDs_.begin(), propRIDs_.begin() + numCoSafeProps_);
    else
        return std::vector<unsigned int>(propRIDs_.begin() + numCoSafeProps_, propRIDs_.end());
}   

void MyPropDecomposition::addProposition(int propRID, bool isSafety)
{
    propRIDs_.push_back(propRID);
    if (isSafety)
        numSafeProps_++;
    else
        numCoSafeProps_++;
}

int MyPropDecomposition::getProposition(int index)
{
    return propRIDs_[index];
}

int MyPropDecomposition::getNumObstacles() const
{
    return obRIDs_.size();
}

void MyPropDecomposition::addObstacles(int obRID)
{
    obRIDs_.push_back(obRID);
}

int MyPropDecomposition::getObstacle(int index)
{
    return obRIDs_[index];
}   

int MyPropDecomposition::regionStatus(int rid)
{
    // Check if the region is a proposition
    for (int i = 0; i < propRIDs_.size(); ++i)
    {
        if (propRIDs_[i] == rid)
            return 1;
    }

    // Check if the region is an obstacle
    for (int i = 0; i < obRIDs_.size(); ++i)
    {
        if (obRIDs_[i] == rid)
            return 0;
    }

    // Check if region is the neighbor to an obstacle
    std::vector<int> neighbors;
    decomp_->getNeighbors(rid, neighbors);
    for (int i = 0; i < neighbors.size(); ++i)
    {
        for (int j = 0; j < obRIDs_.size(); ++j)
        {
            if (neighbors[i] == obRIDs_[j])
                return -2;
        }
    }

    // Otherwise, return -1
    return -1;
}

std::vector<int> MyPropDecomposition::getGridCoord(int rid, int length)
{
    std::vector<int> coord;

    coord.resize(decomp_->getDimension());
    for (int i = decomp_->getDimension() - 1; i >= 0; --i)
    {
        int remainder = rid % length;
        coord[i] = remainder;
        rid /= length;
    }
    return coord;
}