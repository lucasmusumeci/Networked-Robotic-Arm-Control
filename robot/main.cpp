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

//    float q[7];
//    for (int i=0; i < 6; i++) q[i]=0.0;

    float q[6];
    float qr[6];
    GetHandles(clientID);
    for (int i=0; i < 6;i++)q[i]=0.0;

    double dt = 0.01; // Coppelia's time step

    if (clientID!=-1)
    {
        int nbloop=100;
        simxSynchronous(clientID,true);       // Enable the synchronous mode (Blocking function call)
        simxStartSimulation(clientID, simx_opmode_oneshot);
       
        int offsetTime=simxGetLastCmdTime(clientID)/1000;

        Eigen::VectorXd qf(6); qf << 0.0, 40.5, 35.0, 0.0, 59.5, 0.0;
        qf = qf * M_PI / 180.0;  // Convert to radians
        robot.simuTrapezePosition(clientID, handles, qf, 1.0, dt);

        sleep(1);
  
        auto [T, MT] = robot.MGD();

        Eigen::Vector3d Pd; Pd << T(1,3), -T(0,3), T(2,3);
        Eigen::Matrix3d Ad = T.block<3,3>(0,0);
        Eigen::Vector3d xdot; xdot << -0.1, -0.1, 0.0;
        Eigen::Vector3d omega_dot = Eigen::Vector3d::Zero();
        double Kp = 1.0;
        double K0 = 1.0;
        double lambda_L = 0.1;

        robot.cmdCinematique(clientID, handles, Pd, Ad, xdot, omega_dot, Kp, K0, dt, lambda_L);

        Pd << -T(0,3), T(1,3), T(2,3);
        xdot << -0.1, 0.1, 0.0;
        robot.cmdCinematique(clientID, handles, Pd, Ad, xdot, omega_dot, Kp, K0, dt, lambda_L);

        simxStopSimulation(clientID, simx_opmode_oneshot);

        // Close the connection to the server
        simxFinish(clientID);
    }
    else
        printf("Connection to the server not possible\n");

    return(0);
}
