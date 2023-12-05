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

        void addProposition(int prop);

    private:
        std::vector<int> propRIDs_;
};