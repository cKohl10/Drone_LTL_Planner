#include "MyPropDecomposition.h"

MyPropDecomposition::MyPropDecomposition(std::shared_ptr<oc::Decomposition> decomp)
    : oc::PropositionalDecomposition(decomp)
{
    // Constructor implementation...
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

void MyPropDecomposition::addProposition(int propRID)
{
    propRIDs_.push_back(propRID);
}