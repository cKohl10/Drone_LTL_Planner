#include "MyGridDecomposition.h"

MyGridDecomposition::MyGridDecomposition(int len, int dim, const ob::RealVectorBounds &b)
    : oc::GridDecomposition(len, dim, b) 
{
    // Open tree log so each step can be saved when project is called
    tree_log.open("tree_log.txt");
}

void MyGridDecomposition::project(const ob::State* s, std::vector<double>& coord) const
{
    coord.resize(2);
    coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
    coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
    tree_log << coord[0] << ", " << coord[1] <<std::endl;
}

void MyGridDecomposition::sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const
{
    sampler->sampleUniform(s);
    auto* ws = s->as<ob::SE2StateSpace::StateType>();
    ws->setXY(coord[0], coord[1]);
}

void MyGridDecomposition::print()
{
    std::cout << "MyGridDecomposition:" << std::endl;
    std::cout << "Dimensions: " << getDimension() << std::endl;
    std::cout << "Number of Regions: " << getNumRegions() << std::endl;

    for (int i = 0; i < getNumRegions(); i++)
    {
        std::cout << " - Region " << i << " Coordinates: ";
        std::vector<int> coord;
        regionToGridCoord(i, coord);
        for (int j = 0; j < coord.size(); j++)
        {
            std::cout << coord[j] << " ";
        }
        std::cout << std::endl <<" - Region " << i <<" Volume: " << getRegionVolume(i) << std::endl;

        //Output the bounds of the region
        std::cout << " - Region " << i << " Bounds: ";
        const ob::RealVectorBounds& bounds = getRegionBounds(i);
        for (int j = 0; j < bounds.low.size(); j++)
        {
            std::cout << "Dim " << j+1 << ": (" << bounds.low[j] << ", " << bounds.high[j] << ") ";
        }
        std::cout << std::endl;
    }
}