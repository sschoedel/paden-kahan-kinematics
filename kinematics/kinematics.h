#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <iostream>
#include <cmath>
#include <../Eigen/Core>
#include <../Eigen/Geometry>

using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::VectorXf;
using Eigen::Quaternionf;

class Kinematics
{
private:
    int numJoints;
    Matrix4f desEndPosOri;
    Matrix4f curEndPosOri;
    VectorXf homeThetas;  // In radians
    VectorXf curThetas;   // In radians
    VectorXf desThetas;   // In Radians
    std::vector< VectorXf > allDesThetas; // All possible solutions to IK
    std::vector<float> aOff;        // In meters
    std::vector<float> dOff;        // In meters

    // Screw parameters
    std::vector<VectorXf> screwVectors;
    std::vector<VectorXf> screwPoints;
    std::vector<VectorXf> screwTwists;
    
    Matrix4f solve6DofFK(VectorXf inputEndPosOri);
    std::vector< VectorXf > solve6DofIK(Matrix4f inputEndPosOri);

    Matrix4f solve7DofFK(VectorXf inputEndPosOri);
    std::vector< VectorXf > solve7DofIK(Matrix4f inputEndPosOri);
public:
    Kinematics();
    Kinematics(std::vector<float> aOff, std::vector<float> dOff, VectorXf homeThetas);

    Matrix4f solveForwardKinematics(VectorXf inputEndPosOri);
    std::vector< VectorXf > solveInverseKinematics(Matrix4f inputEndPosOri);

    VectorXf chooseOptimalSolution(std::vector< VectorXf > allDesThetas);

    void printJointAngles();
    void printEndPosOri();
};

#endif // KINEMATICS_H