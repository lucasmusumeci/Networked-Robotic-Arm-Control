//
//   C++ - VREP API for manipulator Robotis H
//
//
//
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include "robManip.hpp"

using namespace std;

extern "C" {
    #include "extApi.h"
}

int handles[6],all_ok=1;
simxInt handle, error;

void GetHandles(int clientID){
	simxChar objectName[100];
	char str[10];
    for (int i=0; i < 6; i++) {
        strcpy(objectName, "joint");
        sprintf(str, "%d", i+1);
        strcat(objectName,str);
        error=simxGetObjectHandle(clientID, objectName, &handle, simx_opmode_oneshot_wait);
        if (error == simx_return_ok)
            handles[i]=handle;
        else {
            printf("Error in Object Handle - joint number %d\n", i);
            all_ok=0;
        }
    }
}

/*
 * Communication with simulator
 */
void sendCmd(int clientID, int *handles, const Eigen::VectorXd& q, CmdType_t cmdType) {
    for (int i = 0; i < q.size(); i++) {
        if (cmdType == POSITION) {
            // Implementation for sending position command
            simxSetJointTargetPosition(clientID, handles[i], q(i), simx_opmode_oneshot);
        }
        else if (cmdType == VELOCITY) {
            // Implementation for sending velocity command
            simxSetJointTargetVelocity(clientID, handles[i], q(i), simx_opmode_oneshot);
        }
    }
    // Trigger a simulation step
    simxSynchronousTrigger(clientID);
}

void getJointPosition(int clientID, int jointHandle, double* position) {
    // Implementation for getting joint position
    simxFloat mesured_q;
    simxGetJointPosition(clientID, jointHandle, &mesured_q, simx_opmode_buffer);
    *position = static_cast<double>(mesured_q);
}

int main(int argc,char* argv[])
{

    Robot robot = CreateRobotisH(Eigen::VectorXd::Zero(6));

    Eigen::Matrix4d T = robot.MGD().first;
    cout << "Transformation matrix T:\n" << T << endl;

    Eigen::Vector3d P = T.block<3,1>(0, 3);
    cout << "Jacobian at P:\n" << robot.Jacobienne(P) << endl;

    int portNb=5555;            // the port number where to connect
    int timeOutInMs=5000;       // connection time-out in milliseconds (for the first connection)
    int commThreadCycleInMs=5;  // indicate how often data packets are sent back and forth - a default value of 5 is recommended
    int err;

    // Connection to the server
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,timeOutInMs,commThreadCycleInMs);

    GetHandles(clientID);
    for (int i=0; i < 6;i++)q[i]=0.0;

    double dt = 0.025; // Coppelia's time step

    CmdType_t cmdType = VELOCITY;

    // Initialize streaming for joint positions (no blocking)
    for (int i = 0; i < 6; i++) {
        simxGetJointPosition(clientID, handles[i], nullptr, simx_opmode_streaming);
    }

    if (clientID!=-1)
    {
        int nbloop=100;
        simxSynchronous(clientID,true);       // Enable the synchronous mode (Blocking function call)
        simxStartSimulation(clientID, simx_opmode_oneshot);
       
        int offsetTime=simxGetLastCmdTime(clientID)/1000;

        Eigen::VectorXd qf(6); qf << 0.0, 40.5, 35.0, 0.0, 59.5, 0.0;
        qf = qf * M_PI / 180.0;  // Convert to radians
        robot.simuTrapeze(clientID, handles, qf, 3.0, dt, cmdType);

        sleep(1);
  
        auto [T, MT] = robot.MGD();

        Eigen::Vector3d Pd; Pd << -T(1,3), -T(0,3), T(2,3);
        Eigen::Matrix3d Ad = T.block<3,3>(0,0);
        Eigen::Vector3d xdot; xdot << -0.01, -0.01, 0.0;
        //Eigen::Vector3d xdot; xdot << 0.0, 0.0, 0.0;
        Eigen::Vector3d omega_dot = Eigen::Vector3d::Zero();
        double Kp = 0.2;
        double K0 = 0.5;
        double lambda_L = 0.1;
        double alpha = -2;

        robot.cmdCinematique(clientID, handles, Pd, Ad, xdot, omega_dot, Kp, K0, dt, alpha, lambda_L, cmdType);

        Pd << -T(0,3), T(1,3), T(2,3);
        xdot << -0.01, 0.01, 0.0;
        //xdot << 0.0, 0.0, 0.0;
        robot.cmdCinematique(clientID, handles, Pd, Ad, xdot, omega_dot, Kp, K0, dt, alpha, lambda_L, cmdType);

        simxStopSimulation(clientID, simx_opmode_oneshot);

        // Close the connection to the server
        simxFinish(clientID);
    }
    else
        printf("Connection to the server not possible\n");

    return(0);
}
