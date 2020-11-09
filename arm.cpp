#include "kinematics.h"
#include <stdio.h>

int main()
{
    std::vector<float> aOff, dOff;
    VectorXf homeThetas(6);
    homeThetas[0] = 1;
    homeThetas[1] = 2;
    homeThetas[2] = 1;
    homeThetas[3] = 0;
    homeThetas[4] = 2;
    homeThetas[5] = 3;
    aOff = {0, 0.2, 0.2, 0, 0, 0};
    dOff = {0, 0, 0, 0, 0, 0.1};
    Kinematics k(aOff, dOff, homeThetas);

    Matrix4f inputEndPosOri;
    inputEndPosOri(3,0) = 0;
    inputEndPosOri(3,1) = 0;
    inputEndPosOri(3,2) = 0;
    inputEndPosOri(3,3) = 1;

    inputEndPosOri(0,0) = -0.1801224;
    inputEndPosOri(0,1) = 0.1008298;
    inputEndPosOri(0,2) = 0.9784627;
    inputEndPosOri(1,0) = 0.8432680;
    inputEndPosOri(1,1) = 0.5279511;
    inputEndPosOri(1,2) = 0.1008298;
    inputEndPosOri(2,0) = -0.5064138;
    inputEndPosOri(2,1) = 0.8432680;
    inputEndPosOri(2,2) = -0.1801224;

    inputEndPosOri(0,3) = .05;
    inputEndPosOri(1,3) = .05;
    inputEndPosOri(2,3) = .05;

    std::vector<VectorXf> sols = k.solveInverseKinematics(inputEndPosOri);
    VectorXf sol = sols[0];
    std::cout << "sol: " << std::endl << sol << std::endl;
    k.printJointAngles();
    k.printEndPosOri();
}