#include "kinematics.h"

Kinematics::Kinematics()
{
    numJoints = 6; // Default to 6R
    float defaultLinkLen = (float)0.1;
    float defaultLinkOffset = (float)0.0;
    curEndPosOri.setConstant(0); // end effector position and orientation matrix to zero
    for (int i=0; i<numJoints; i++)
    {
        // Set default initial joint angles and lengths
        homeThetas[i] = 0;
        desThetas[i] = 0;
        curThetas[i] = 0;
        aOff.push_back(defaultLinkOffset);
        dOff.push_back(defaultLinkLen);

        // // Set default screw parameters
        // VectorXf standardUnitVec(0,0,1);
        // screwVectors.push_back(standardUnitVec);
        // VectorXf standardPoint(0,0,dOff[i]);
        // screwPoints.push_back(standardPoint);
        // VectorXf screwTwists(6);
        // // for (int i=0; i<6; i++)
        // // {

        // // }
        // // screwTwists.push_back();
    }
}

Kinematics::Kinematics(std::vector<float> aOff, std::vector<float> dOff, VectorXf homeThetas)
{
    // Set initial joint angles and lengths
    this->aOff = aOff;
    this->dOff = dOff;
    this->homeThetas = homeThetas;
    curThetas = homeThetas;
    desThetas = homeThetas;
    numJoints = static_cast<int>(homeThetas.size());
    curEndPosOri.setConstant(0); // End effector position and orientation matrix to zero
    curEndPosOri = solveForwardKinematics(homeThetas); // Calculate initial current and desired end effector position and orientation
    desEndPosOri = curEndPosOri;

    // Set screw parameters
}

// Solve for end effector position given input thetas
Matrix4f Kinematics::solveForwardKinematics(VectorXf inputThetas)
{
    if (numJoints == 6)
        return solve6DofFK(inputThetas);
    else if (numJoints == 7)
        return solve7DofFK(inputThetas);
    else
        std::cout << "Unsupported robot type. Must be 6R or 7R. Number of joints given: " << numJoints << std::endl;    
    return curEndPosOri;
}

// Solve for thetas given end effector position and orientation
// @return vector of possible solution vectors
std::vector< VectorXf > Kinematics::solveInverseKinematics(Matrix4f inputEndPosOri)
{
    allDesThetas.clear();
    if (numJoints == 6)
        return solve6DofIK(inputEndPosOri);
    else if (numJoints == 7)
        return solve7DofIK(inputEndPosOri);
    else
        std::cout << "Unsupported robot type. Must be 6R or 7R. Number of joints given: " << numJoints << std::endl;    
    // Return home position as only solution if not 6 or 7 joint robot
    std::vector<VectorXf> homeSolution;
    homeSolution.push_back(homeThetas);
    return  homeSolution;
}




////////////////
// 6dof FK and IK
////////////////
Matrix4f Kinematics::solve6DofFK(VectorXf inputThetas)
{
    Matrix4f endPosOri;
    endPosOri.setConstant(0);

    return endPosOri;
}

std::vector< VectorXf > Kinematics::solve6DofIK(Matrix4f inputEndPosOri)
{
    allDesThetas.clear(); // Store each possible solution here
    Matrix3f R = inputEndPosOri.block(0,0,3,3); // End effector rotation matrix
    VectorXf endPos = inputEndPosOri.block(0,3,3,1);
    VectorXf wristPos = endPos - R.block(0,2,3,1) * dOff[5];

    // std::cout << "R: " << std::endl << R << std::endl;
    // std::cout << "R.block(0,2,3,1): " << std::endl << R.block(0,2,3,1) << std::endl;
    // std::cout << "endPos: " << std::endl << endPos << std::endl;
    // std::cout << "wristPos: " << std::endl << wristPos << std::endl;

    float c3 = (pow(wristPos[0], 2) + pow(wristPos[1], 2) + pow(wristPos[2], 2) - pow(aOff[1], 2) - pow(aOff[2], 3)) / (2*aOff[1]*aOff[2]);
    float s3 = -sqrt(1-pow(c3, 2));
    float theta3 = atan2(s3, c3);

    float c2 = ( sqrt(pow(wristPos[0], 2) + pow(wristPos[1], 2)) * (aOff[1] + aOff[2] * c3) + wristPos[2] * aOff[2] * s3 ) / (pow(aOff[1], 2) + pow(aOff[2], 2) + 2*aOff[1]*aOff[2]*c3);
    float s2 = (wristPos[2]*(aOff[1]+aOff[2]*c3) - sqrt(pow(wristPos[0],2) + pow(wristPos[1],2)) * aOff[2] * s3) / (pow(aOff[1],2) + pow(aOff[2],2) + 2*aOff[1]*aOff[2]*c3);
    float theta2 = atan2((aOff[1]+aOff[2]*c3)*wristPos[2] - aOff[2]*s3*sqrt(pow(wristPos[0],2) + pow(wristPos[1],2)), (aOff[1]+aOff[2]*c3) * sqrt(pow(aOff[0],2) + pow(aOff[1],2)) + aOff[2]*s3*wristPos[2]);

    float theta1 = atan2(wristPos[1],wristPos[1]);

    Matrix3f R3_0;
    R3_0(0,0) = cos(theta1) * cos(theta2 + theta3);
    R3_0(0,1) = -cos(theta1) * sin(theta2 + theta3);
    R3_0(0,2) = sin(theta1);
    R3_0(1,0) = sin(theta1) * cos(theta2 + theta3);
    R3_0(1,1) = -sin(theta1) * sin(theta2 + theta3);
    R3_0(1,2) = -cos(theta1);
    R3_0(2,0) = sin(theta2 + theta3);
    R3_0(2,1) = cos(theta2 + theta3);
    R3_0(2,2) = 0;

    // std::cout << "R3: " << std::endl << R3 << std::endl;

    Matrix3f R6_3 = R3_0.transpose()*R;

    float theta4 = atan2(R6_3(1,2),R6_3(0,2));
    float theta5 = atan2( sqrt(pow(R6_3(0,2), 2) + pow(R6_3(1,2),2) ), R6_3(2,2) );
    float theta6 = atan2(R6_3(2,1), R6_3(2,0));

    VectorXf sol(6);
    sol[0] = theta1;
    sol[1] = theta2;
    sol[2] = theta3;
    sol[3] = theta4;
    sol[4] = theta5;
    sol[5] = theta6;
    allDesThetas.push_back(sol);

    return allDesThetas;
}






////////////////
// 7dof FK and IK
////////////////
Matrix4f Kinematics::solve7DofFK(VectorXf inputThetas)
{
    
    // TODO;
    Matrix4f endPosOri(4,4);
    return endPosOri;
}

std::vector< VectorXf > Kinematics::solve7DofIK(Matrix4f inputEndPosOri)
{
    // TODO;
    return allDesThetas;
}







VectorXf Kinematics::chooseOptimalSolution(std::vector< VectorXf > allDesThetas)
{
    return desThetas;
}

void Kinematics::printJointAngles()
{
    std::cout << "-----Current Joint Angles-----" << std::endl;
    for (int i=0; i<curThetas.size(); i++)
    {
        std::cout << i << ": " << curThetas[i] << std::endl;
    }
    std::cout << std::endl;
}

void Kinematics::printEndPosOri()
{
    Matrix3f endEffectorRotationMat = curEndPosOri.block(0,0,3,3);
    Quaternionf curEndOri(endEffectorRotationMat);

    std::cout << "-----Current End Effector Position-----" << std::endl;
    std::cout << "x: " << curEndPosOri.coeff(0,3);
    std::cout << "  y: " << curEndPosOri.coeff(1,3);
    std::cout << "  z: " << curEndPosOri.coeff(2,3);
    std::cout << std::endl;

    std::cout << "-----Current End Effector Orientation-----" << std::endl;
    std::cout << "Quaternion form: ";
    std::cout << "w: " << curEndOri.w();
    std::cout << "  x: " << curEndOri.x();
    std::cout << "  y: " << curEndOri.y();
    std::cout << "  z: " << curEndOri.z() << std::endl;
    std::cout << "Rotation matrix form: " << std::endl;
    std::cout << endEffectorRotationMat;
    std::cout << std::endl;
}