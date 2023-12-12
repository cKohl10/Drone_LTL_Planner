#pragma once
#include "includeCore.h"
#include <fstream> // Add this line

namespace ob = ompl::base;
namespace oc = ompl::control;

class MyGridDecomposition : public oc::GridDecomposition
{
    public:
        MyGridDecomposition(int len, int dim, const ob::RealVectorBounds &b);

        ~MyGridDecomposition() override = default;
  
        /** \brief Project a given State to a set of coordinates in R^k, where k is the dimension of this
             * Decomposition. */
        void project(const ob::State* s, std::vector<double>& coord) const override;
    
        /** \brief Samples a State using a projected coordinate and a StateSampler. */
        void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override;

        /**
         * @brief Prints the grid decomposition.
         */
        void print();

    private:
        //Create log file for tree
        mutable std::ofstream tree_log;
};