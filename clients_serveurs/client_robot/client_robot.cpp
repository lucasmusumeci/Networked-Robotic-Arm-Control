#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>

#include <arpa/inet.h>
#include <unistd.h>

#include <sys/time.h>

#include <iostream>
#include <math.h>

#include "robManip.hpp"
#include "logger.hpp"

using namespace std;

/*
 * msg_t and Time functions are already defined in logger.hpp, but in logger.hpp isn't included, you must uncomment it to use it in this file.
 */
/*
#define NB_JOINTS 6
typedef struct {
	int cmdType;
	double cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	double qdot_simu[NB_JOINTS];
	struct timeval time;
}msg_t;
*/
/*
 * Time functions
 */
/*
long long getTimeElapsed_ms(struct timeval * t0)
{
    struct timeval t;
    gettimeofday(&t, NULL);

    return (t.tv_sec - t0->tv_sec)*1000 + (t.tv_usec - t0->tv_usec)/1000;
}

long long diffTime_ms(struct timeval * t0, struct timeval * t)
{
    return (t->tv_sec - t0->tv_sec)*1000 + (t->tv_usec - t0->tv_usec)/1000;
}

long long timeval2ms(struct timeval *t)
{
    return t->tv_sec*1000 + t->tv_usec/1000;
}
*/
/*
 *  Global variables for communication
 */

#define ERROR (-1)

static const double tau[NB_JOINTS] = {0.002, 0.002, 0.003, 0.002, 0.001, 0.001};
static const double G[NB_JOINTS] = {1.00, 1.00, 1.00, 1.00, 1.00, 1.00};
double dt = 0.025; // Coppelia's time step

// Client de 127.0.0.2
int client;
struct sockaddr_in sockAddr_client;
socklen_t addr_client;
msg_t message_client;

// Serveur (127.0.0.3)
int serveur;
struct sockaddr_in sockAddr_serveur;
socklen_t addr_serveur;
msg_t latest_message_serveur; // Stores the response from the server to the latest command sent by the client

int main (int nba, char *arg[])
{
	// Client de 127.0.0.2 (retard)
	client=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_client.sin_family=PF_INET;
	sockAddr_client.sin_port=htons(2002);
	sockAddr_client.sin_addr.s_addr=inet_addr("127.0.0.2");
	addr_client=sizeof(sockAddr_client);

	message_client.cmdType=VELOCITY;

	fcntl(client,F_SETFL,fcntl(client,F_GETFL) | O_NONBLOCK);

	// Serveur (127.0.0.3)
	serveur=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_serveur.sin_family=PF_INET;
	sockAddr_serveur.sin_port=htons(2003);
	sockAddr_serveur.sin_addr.s_addr=inet_addr("127.0.0.3");
	addr_serveur=sizeof(sockAddr_serveur);

	int err_serveur=bind(serveur,(struct sockaddr*)&sockAddr_serveur,addr_serveur);
	if(err_serveur==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	fcntl(serveur,F_SETFL,fcntl(serveur,F_GETFL) | O_NONBLOCK);

	// Initialize the robot model
	Robot robot = CreateRobotisH(Eigen::VectorXd::Zero(6));

    // Desired position
    Eigen::VectorXd qf(6); qf << 120.0, 60.0, -60.0, 120.0, 60.0, 120.0;
    qf = qf * M_PI / 180.0;  // Convert to radians

	//  --- Adaptive control loop ---
    Eigen::VectorXd q_mes(NB_JOINTS);
    Eigen::VectorXd q_cmd(NB_JOINTS);
    
	// Safety Gain Margin (ideally between 0.2 and 0.6) the higher, the more oscillations
	// /!\ Must be <1 in our case to ensure stability (KcG < 1)
    double alpha = 0.4;
    double max_err_treshold = 0.001; // radians

    printf("Starting adaptive control loop...\n");

    Logger logger("adaptive_control_log.csv");
	int position_ok = 0;
	int exit = false;

    int timeout_ms = 10000; // Max 10 seconds to reach the target
    struct timeval start;
    gettimeofday(&start, NULL);
    struct timeval now;
    gettimeofday(&now, NULL);
    
    while(!exit && diffTime_ms(&start, &now) < timeout_ms) {

        gettimeofday(&now, NULL);

        // Read current positions
        if (getAllJointsPosition(0, 0, &q_mes) == ERROR) {
            continue; // Wait for valid message to arrive
        }

        // Record the current message
        logger.record(&latest_message_serveur);

        // Measure dynamic Trc in seconds
        struct timeval now;
        gettimeofday(&now, NULL);
        double Trc = diffTime_ms(&latest_message_serveur.time, &now) / 1000.0;
		printf("Measured Trc: %f seconds\n", Trc);
        
        if (Trc <= 0.001) Trc = 0.001; // Safety floor to prevent division by zero

        // Compute adaptive Kc and apply control law for each joint
        double max_err = 0.0;
        for (int i = 0; i < NB_JOINTS; i++) {
            // Pulsation critique (omega_u) based on the measured delay
            double omega_u = M_PI / (2.0 * (Trc + tau[i]));
            
            // Calculate stability limit (K_max)
            double K_max = (omega_u / G[i]) * sqrt(1.0 + pow(tau[i] * omega_u, 2));
            
            // Apply safety margin to get adaptive Kc
            double Kc = alpha * K_max;
            
            // Calculate proportional velocity command
            double err = qf(i) - q_mes(i);
            q_cmd(i) = Kc * err;
            
            // Determine the maximum position error between every joint
            if (abs(err) > max_err) max_err = abs(err);
        }

        // Send velocity command
        sendCmd(0, 0, q_cmd, VELOCITY);

        // Break loop if all joints are within 0.001 radians of target
        if (max_err < max_err_treshold) {
			position_ok++;
			switch(position_ok) {
				case 1:
            		printf("Target 1 reached successfully with Tr = %f s\n", Tr);
                    exit = true;
					qf = Eigen::VectorXd::Zero(6);
					break;
                // Can add more cases here for multiple targets if needed
				default:
					exit = true;
            }
        }
    }

    sendCmd(0, 0, Eigen::VectorXd::Zero(6), VELOCITY); // Send null command to stop the robot

    // Wait for pending responses to ensure all commands have been processed before flushing the log to the disk
    logger.waitForPending(serveur, &sockAddr_serveur, &addr_serveur, &latest_message_serveur, 1000);
    printf("Flushing log to disk...\n");
    logger.flush();


	/*
	 *  Commande Trapeze puis cinématique
	 *  /!\ Works better with no correction term Kc because the delay is constant
	 *  For the commande trapeze, the trajectory is pre planned and sent in full to the simulator, so the delay doesn't cause any issue.
	 *  For the cinématique command, it still converges because it's already reboucled
	 */
	/*
	Eigen::VectorXd qf(6); qf << 0.0, 40.5, 35.0, 0.0, 59.5, 0.0;
    qf = qf * M_PI / 180.0;  // Convert to radians

    robot.simuTrapeze(0, 0, qf, 3.0, dt, VELOCITY);

	sleep(10*dt);
  
    auto [T, MT] = robot.MGD();

    Eigen::Vector3d Pd; Pd << -T(1,3), -T(0,3), T(2,3);
    Eigen::Matrix3d Ad = T.block<3,3>(0,0);
    //Eigen::Vector3d xdot; xdot << -0.01, -0.01, 0.0;
    Eigen::Vector3d xdot; xdot << 0.0, 0.0, 0.0;
    Eigen::Vector3d omega_dot = Eigen::Vector3d::Zero();
    double Kp = 0.2;
    double K0 = 0.5;
    double lambda_L = 0.1;
    double alpha = -2;

    robot.cmdCinematique(0, 0, Pd, Ad, xdot, omega_dot, Kp, K0, dt, alpha, lambda_L, VELOCITY);

	Pd << -T(0,3), T(1,3), T(2,3);
    //xdot << -0.01, 0.01, 0.0;
    xdot << 0.0, 0.0, 0.0;
    robot.cmdCinematique(0, 0, Pd, Ad, xdot, omega_dot, Kp, K0, dt, alpha, lambda_L, VELOCITY);

    sendCmd(0, 0, Eigen::VectorXd::Zero(6), VELOCITY); // Send null command to stop the robot
	*/

	close(serveur);
	close(client);

	return 0;
}

/*
 * sendCmd implementation : the clientID and hanbdles are handled by the server
 * The functions in robManip.cpp call this function to send commands
 */
void sendCmd(int clientID, int *handles, const Eigen::VectorXd& q, CmdType_t cmdType)
{
	// Empty the receive buffer and stores the latest message. This prevents the stacking up of old messages
    msg_t tmp = {};
    while (recvfrom(serveur, &tmp, sizeof(tmp), 0,
                    (struct sockaddr*)&sockAddr_serveur, &addr_serveur) != ERROR)
    {
        if (diffTime_ms(&latest_message_serveur.time, &tmp.time) > 0)
            latest_message_serveur = tmp;
    }

    message_client.cmdType = cmdType;
    gettimeofday(&message_client.time, NULL);

	for (int i = 0; i < NB_JOINTS; i++)
	{		
		message_client.cmd[i] = q(i);
	}

    // Send the command to retard_robot
    sendto(client, &message_client, sizeof(message_client), 0,
           (struct sockaddr*)&sockAddr_client, sizeof(sockAddr_client));

    usleep(dt * 1000000); // Wait for dt
}

/*
 * Used in cmdCinematique to get the latest joint positions from the simulator. Returns -1 on error, 0 on success.
 */
int getAllJointsPosition(int clientID, int *handles, Eigen::VectorXd *theta)
{
    msg_t tmp;
    int resultr = ERROR;

	// Check if there are any new messages and keep the latest one
    while (recvfrom(serveur, &tmp, sizeof(tmp), 0,(struct sockaddr*)&sockAddr_serveur, &addr_serveur) != ERROR)
    {
        resultr = 0;
        if (diffTime_ms(&latest_message_serveur.time, &tmp.time) > 0)
            latest_message_serveur = tmp;
    }

    // If we never received any message, return an error
    if (resultr == ERROR && latest_message_serveur.time.tv_sec == 0)
    {
        printf("getAllJointsPosition: no data received from server yet\n");
        return -1;
    }

    // We have the latest message from the server in latest_message_serveur, so we can extract joint positions from it
    for (int i = 0; i < NB_JOINTS; i++)
        (*theta)[i] = latest_message_serveur.q_simu[i];

    return 0;
}

/*
 * Not used for now :(
 */
/*
int getAllJointsVelocity(int clientID, int *handles, Eigen::VectorXd *theta_dot)
{
    msg_t tmp;
    int resultr = ERROR;

	// Check if there are any new messages and keep the latest one
    while (recvfrom(serveur, &tmp, sizeof(tmp), 0,(struct sockaddr*)&sockAddr_serveur, &addr_serveur) != ERROR)
    {
        resultr = 0;
        if (diffTime_ms(&latest_message_serveur.time, &tmp.time) > 0)
            latest_message_serveur = tmp;
    }

	// If we never received any message, return an error
    if (resultr == ERROR && latest_message_serveur.time.tv_sec == 0)
    {
        // No data ever received yet — genuine error
        printf("getAllJointsPosition: no data received from server yet\n");
        return -1;
    }

    for (int i = 0; i < NB_JOINTS; i++)
        (*theta_dot)[i] = latest_message_serveur.qdot_simu[i];

    return 0;
}
*/