#include "kinematics.h"

kinematics::kinematics()
{
    for (int i=0; i<6; i++)
    {
        homeThetas.push_back(0);
        desThetas.push_back(homeThetas[i]);
        curThetas.push_back(homeThetas[i]);
        aOff.push_back(defaultLinkOffset);
        dOff.push_back(defaultLinkLen);
    }
}

kinematics::kinematics(std::vector<float> aOff, std::vector<float> dOff, std::vector<float> homeThetas)
{
    this->aOff = aOff;
    this->dOff = dOff;
    this->homeThetas = homeThetas;
    curThetas = homeThetas;
    desThetas = homeThetas;
}

// Solve for end effector position given input thetas
std::vector<float> kinematics::solveForwardKinematics(std::vector<float> inputThetas)
{
    MatrixXf m(2,2);
    return desEndPosOri;
}

// Solve for thetas given end effector position and orientation
// @return vector of possible solution vectors
std::vector< std::vector<float> > kinematics::solveInverseKinematics(std::vector<float> inputEndPosOri)
{
    allDesThetas.clear();

    return allDesThetas;
}