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
	double q_cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	double qdot_simu[NB_JOINTS];
	struct timeval time;
}msg_t;

#define ERROR (-1)

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
msg_t message_serveur;

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

	sleep(1);
  
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

	close(serveur);

	return 0;

}

/*
 * Communication
 */
void sendCmd(int clientID, int *handles, const Eigen::VectorXd& q, CmdType_t cmdType)
{
	for (int i = 0; i < NB_JOINTS; i++) {
		message_client.q_cmd[i] = q(i);
	}
	message_client.cmdType = cmdType;
	gettimeofday(&message_client.time, NULL);

	int results = sendto(client,&message_client,sizeof(message_client),0,(struct sockaddr*)&sockAddr_client,sizeof(sockAddr_client));

	//printf("--- client --- \n  rr=%d rs=%d\n time=%ld.%ld\n ",resultr, results, message.time.tv_sec,message.time.tv_usec);

	usleep(dt*1000000); // Sleep for the duration of the time step (replace syncrhonous mode in Coppelia)
}

int getAllJointsPosition(int clientID, int *handles, Eigen::VectorXd *theta)
{


	int resultr = ERROR;
	
	while(recvfrom(serveur,&message_serveur,sizeof(message_serveur), 0,(struct sockaddr*)&sockAddr_serveur,&addr_serveur) != ERROR)
	{
		resultr = 0; // Received at least 1 message
	}

	if (resultr == ERROR)
	{
		printf("Error receiving joint positions from server\n");
		return -1; // Return error
	}
	
	//double *q_simu = message_serveur.q_simu;
	//printf("q = [%f , %f , %f , %f , %f , %f]",q_simu[0],q_simu[1],q_simu[2],q_simu[3],q_simu[4],q_simu[5]);
	for(int i=0 ; i<NB_JOINTS ; i++)
	{
		(*theta)[i] = message_serveur.q_simu[i];
	}

	printf("theta = [");
    for (int i = 0; i < 6; ++i) {
        printf("%f", (*theta)(i));
        if (i < 5) printf(", ");
    }
    printf("]\n");	

    return 0; // Return success
}
