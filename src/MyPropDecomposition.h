#pragma once
#include "includeCore.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class MyPropDecomposition : public oc::PropositionalDecomposition
{
    public:
        MyPropDecomposition(std::shared_ptr<oc::Decomposition> decomp);

        ~MyPropDecomposition() override = default;

        oc::World worldAtRegion(int rid) override;

        int getNumProps() const override;

        int getNumCoSafeProps() const;

        std::vector<unsigned int> getPropositions(bool isSafety);

        // Add a proposition to the list of propositions region ids
        void addProposition(int prop, bool isSafety);

        // Get the region id corresponding to the proposition number
        int getProposition(int index);

        // Get the number of obstacles
        int getNumObstacles() const;

        // Add obstacles to the list of obstacles region ids
        void addObstacles(int obRID);

        // Get the region id corresponding to the obstacle number
        int getObstacle(int index);

        // Check if a given region is an obstacle or a proposition
        // Returns 1 if the region is a proposition
        // Returns 0 if the region is an obstacle
        // Returns -1 if the region is neither
        int regionStatus(int rid);

        std::vector<int> getGridCoord(int rid, int length);

        // Create a grid 

    private:
        int numCoSafeProps_;
        int numSafeProps_;
        std::vector<unsigned int> propRIDs_;
        std::vector<unsigned int> obRIDs_;
};