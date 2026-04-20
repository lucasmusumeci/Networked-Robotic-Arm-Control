/*
 * step_ident.cpp — Per-joint step identification via direct CoppeliaSim API
 *
 * Communicates directly with the simulator (no UDP delay emulator) so that
 * the measured response reflects only the true actuator dynamics
 * 
 * Make sure to set the CoppeliaSim time step to 1ms (dt=0.001)
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include "robManip.hpp"
#include "logger.hpp"

extern "C" {
    #include "extApi.h"
}

using namespace std;

/* ------------------------------------------------------------------ */
/*  Experiment parameters — adjust before running                     */
/* ------------------------------------------------------------------ */

// Joints to identify
static const int JOINTS_TO_IDENTIFY[] = {1, 2, 3, 4, 5, 6};
static const int N_IDENT = 6;

// Velocity step amplitude per joint (rad/s).
// Keep small enough to stay well within joint limits.
static const double V_STEP[6] = {0.05, 0.05, 0.1, 0.1, 0.1, 0.1};

// Duration of the velocity step (s).
static const double T_EXCITE = 0.04;

// Duration of the zero-velocity phase before the step (s)
static const double T_REST = 0.005;

// Home pose — robot is moved here before every experiment.
static const double HOME_DEG[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// /!\ Must match the simulation time step set in CoppeliaSim /!\.
static const double DT = 0.001;

/* ------------------------------------------------------------------ */
/*  CoppeliaSim handles (global, filled by GetHandles)                */
/* ------------------------------------------------------------------ */

#define ERROR (-1)

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

/* ------------------------------------------------------------------ */
/*  Direct-API communication (no UDP, no delay)                       */
/* ------------------------------------------------------------------ */

void sendCmd(int clientID, int *h, const Eigen::VectorXd& q, CmdType_t cmdType)
{
    for (int i = 0; i < q.size(); i++) {
        if (cmdType == POSITION)
            simxSetJointTargetPosition(clientID, h[i], (simxFloat)q(i),
                                       simx_opmode_oneshot);
        else if (cmdType == VELOCITY)
            simxSetJointTargetVelocity(clientID, h[i], (simxFloat)q(i),
                                       simx_opmode_oneshot);
    }
    simxSynchronousTrigger(clientID);
}

int getAllJointsPosition(int clientID, int *h, Eigen::VectorXd *theta)
{
    for (int i = 0; i < 6; i++) {
        simxFloat mq;
        if (simxGetJointPosition(clientID, h[i], &mq, simx_opmode_buffer)
                == simx_return_ok)
            (*theta)(i) = static_cast<double>(mq);
        else {
            printf("Error reading position for joint %d\n", i);
            return -1;
        }
    }
    return 0;
}

int getAllJointsVelocity(int clientID, int *h, Eigen::VectorXd *theta_dot)
{
    for (int i = 0; i < 6; i++) {
        simxFloat mdq;
        if (simxGetObjectFloatParameter(clientID, h[i], 2012, &mdq, simx_opmode_buffer) == simx_return_ok)
            (*theta_dot)(i) = static_cast<double>(mdq);
        else {
            printf("Error reading velocity for joint %d\n", i);
            return -1;
        }
    }
    return 0;
}


/* ------------------------------------------------------------------ */
/*  Single-joint step experiment                                      */
/* ------------------------------------------------------------------ */

static void run_step_experiment(int clientID, Robot& robot, int joint_idx)
{
    const int K_excite = (int)(T_EXCITE / DT);
    const int K_rest   = (int)(T_REST   / DT);

    string fname = "joint" + to_string(joint_idx+1) + "_step.csv";
    StepLogger logger(fname, joint_idx, clientID);

    printf("\n=== Joint %d: step %.3f rad/s for %.1f s ===\n",
           joint_idx+1, V_STEP[joint_idx], T_EXCITE);

    Eigen::VectorXd qdot_cmd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_simu   = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qdot_simu   = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd old_qdot_simu = Eigen::VectorXd::Zero(6); // Fallback in case of read errors
    
    // ---- Phase 1: zero velocity  ----
    qdot_cmd(joint_idx) = 0.0;

    for (int k = 0; k < K_rest; k++) {
        sendCmd(clientID, handles, qdot_cmd, VELOCITY);

        if (getAllJointsPosition(clientID, handles, &q_simu) == -1)
            q_simu = robot.getTheta();   // fallback to last known state

        robot.setTheta(q_simu);

        if (getAllJointsVelocity(clientID, handles, &qdot_simu) == -1)
            qdot_simu = old_qdot_simu;   // fallback to last known state

        logger.record(qdot_cmd, qdot_simu);
        old_qdot_simu = qdot_simu;
    }

    // ---- Phase 2: velocity step ----
    qdot_cmd(joint_idx) = V_STEP[joint_idx];    // Still zero for other joints

    for (int k = 0; k < K_excite; k++) {
        sendCmd(clientID, handles, qdot_cmd, VELOCITY);

        if (getAllJointsPosition(clientID, handles, &q_simu) == -1)
            q_simu = robot.getTheta();   // fallback to last known state

        robot.setTheta(q_simu);

        if (getAllJointsVelocity(clientID, handles, &qdot_simu) == -1)
            qdot_simu = old_qdot_simu;   // fallback to last known state

        logger.record(qdot_cmd, qdot_simu);
        old_qdot_simu = qdot_simu;
    }

    logger.flush();
    printf("    Written to %s\n", fname.c_str());
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    // ---- Connect to CoppeliaSim ----
    const int portNb              = 5555;
    const int timeOutInMs         = 5000;
    const int commThreadCycleInMs = 5;

    int clientID = simxStart((simxChar*)"127.0.0.1", portNb,
                             true, true, timeOutInMs, commThreadCycleInMs);
    if (clientID == -1) {
        printf("Connection to CoppeliaSim not possible\n");
        return -1;
    }

    GetHandles(clientID);
    if (!all_ok) {
        printf("Could not get all joint handles — aborting.\n");
        simxFinish(clientID);
        return -1;
    }

    // Synchronous mode: every simxSynchronousTrigger advances exactly one simulation step (dt=1ms)
    simxSynchronous(clientID, true);
    simxStartSimulation(clientID, simx_opmode_oneshot);

    // Initialize position streaming (required before simx_opmode_buffer reads)
    for (int i = 0; i < 6; i++) {
        simxGetJointPosition(clientID, handles[i], nullptr, simx_opmode_streaming);
        simxGetObjectFloatParameter(clientID, handles[i], 2012, nullptr, simx_opmode_streaming);
    }
    // ---- Robot model ----
    Robot robot = CreateRobotisH(Eigen::VectorXd::Zero(6));

    // ---- Starting at home pose ----
    Eigen::VectorXd qhome(6);
    for (int i = 0; i < 6; i++)
        qhome(i) = HOME_DEG[i] * M_PI / 180.0;

    robot.setTheta(qhome);
    Eigen::VectorXd q_init = Eigen::VectorXd::Zero(6);
    q_init = qhome;

    // ---- Run experiments ----
    for (int n = 0; n < N_IDENT; n++) {

        if (n > 0) {
            printf("Returning to home before joint %d...\n", n+1);
            robot.simuTrapeze(clientID, handles, qhome, 0.0, DT, VELOCITY);
            sleep(1);
            if (getAllJointsPosition(clientID, handles, &q_init) == 0)
                robot.setTheta(q_init);
        }

        run_step_experiment(clientID, robot, n);
    }

    printf("\nAll experiments complete.\n");

    simxStopSimulation(clientID, simx_opmode_oneshot);
    simxFinish(clientID);
    return 0;
}
