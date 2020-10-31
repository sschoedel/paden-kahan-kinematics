#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Eigen::MatrixXf;

class kinematics
{
private:
    float defaultLinkLen = (float)0.1;
    float defaultLinkOffset = (float)0.0;
    std::vector<float> desEndPosOri;
    std::vector<float> curEndPosOri;
    std::vector<float> homeThetas;  // In radians
    std::vector<float> curThetas;   // In radians
    std::vector<float> desThetas;   // In Radians
    std::vector< std::vector<float> > allDesThetas; // All possible solutions to IK
    std::vector<float> aOff;        // In meters
    std::vector<float> dOff;        // In meters
public:
    kinematics();
    kinematics(std::vector<float> aOff, std::vector<float> dOff, std::vector<float> homeThetas);

    std::vector<float> solveForwardKinematics(std::vector<float> inputThetas);
    std::vector< std::vector<float> > solveInverseKinematics(std::vector<float> inputEndPosOri);
};

#endif // KINEMATICS_H