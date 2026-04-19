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

using namespace std;

#define NB_JOINTS 6
typedef struct {
	int cmdType;
	double cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	double qdot_simu[NB_JOINTS];
	struct timeval time;
}msg_t;

#define ERROR (-1)


static const double tau[NB_JOINTS] = {0.002, 0.002, 0.003, 0.002, 0.001, 0.001};
static const double Kc[NB_JOINTS] = {0.8, 0.8, 0.7, 0.5, 0.4, 0.4};

/*
 * Time functions
 */
int getTimeElapsed_ms(struct timeval * t0)
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

/*
 *  Global variables for communication
 */

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
msg_t latest_message_serveur; // Store the response from the server to the latest command sent by the client

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

	close(serveur);
	close(client);

	return 0;

}

/*
 * Communication
 */
void sendCmd(int clientID, int *handles, const Eigen::VectorXd& q, CmdType_t cmdType)
{
    message_client.cmdType = cmdType;
    gettimeofday(&message_client.time, NULL);

    for (int i = 0; i < NB_JOINTS; i++)
    {
        switch (cmdType)
        {
        case POSITION:
            message_client.cmd[i] = q(i);
            break;
        case VELOCITY:
            message_client.cmd[i] = q(i);
            break;
        }
    }

    sendto(client, &message_client, sizeof(message_client), 0,
           (struct sockaddr*)&sockAddr_client, sizeof(sockAddr_client));

    // Drain the receive buffer right after sending, while the simulator
    // is processing the command. This prevents frames from piling up
    // in the socket buffer and causing the burst-read behaviour.
	int drain_count = 0;
    msg_t tmp = {};
    while (recvfrom(serveur, &tmp, sizeof(tmp), 0,
                    (struct sockaddr*)&sockAddr_serveur, &addr_serveur) != ERROR)
    {
		drain_count++;
        if (diffTime_ms(&latest_message_serveur.time, &tmp.time) > 0)
            latest_message_serveur = tmp;
    }

	printf("sendCmd: drained %d packets\n", drain_count);

    usleep(dt * 1000000); // give the simulator time to respond
}

int getAllJointsPosition(int clientID, int *handles, Eigen::VectorXd *theta)
{
    // Drain any remaining frames that arrived during usleep.
    // sendCmd already keeps this buffer thin, so usually 0-1 frames land here.
    msg_t tmp;
    int resultr = ERROR;

    while (recvfrom(serveur, &tmp, sizeof(tmp), 0,(struct sockaddr*)&sockAddr_serveur, &addr_serveur) != ERROR)
    {
        resultr = 0;
        if (diffTime_ms(&latest_message_serveur.time, &tmp.time) > 0)
            latest_message_serveur = tmp;
    }

    // Accept stale data from a previous sendCmd drain — the control loop
    // can still use the last known good state rather than failing entirely.
    if (resultr == ERROR && latest_message_serveur.time.tv_sec == 0)
    {
        // No data ever received yet — genuine error
        printf("getAllJointsPosition: no data received from server yet\n");
        return -1;
    }

    for (int i = 0; i < NB_JOINTS; i++)
        (*theta)[i] = latest_message_serveur.q_simu[i];

    return 0;
}
