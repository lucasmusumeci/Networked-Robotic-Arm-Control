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
	double q_cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	int cmdType;
	struct timeval time;
}msg_t;

#define ERROR (-1)

/*
 *  Global variables for communication
 */

int serveur;
struct sockaddr_in sockAddr;
socklen_t addr;
msg_t message;
double dt = 0.025; // Coppelia's time step

int main (int nba, char *arg[])
{
	// Initialize the robot model
	Robot robot = CreateRobotisH(Eigen::VectorXd::Zero(6));

    Eigen::Matrix4d T = robot.MGD().first;
    cout << "Transformation matrix T:\n" << T << endl;

    Eigen::Vector3d P = T.block<3,1>(0, 3);
    cout << "Jacobian at P:\n" << robot.Jacobienne(P) << endl;

	// Client
	long int Te;

	serveur=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr.sin_family=PF_INET;
	sockAddr.sin_port=htons(2001);
	sockAddr.sin_addr.s_addr=inet_addr("127.0.0.2");
	addr=sizeof(sockAddr);

	message.cmdType=VELOCITY;
	Te=100000; // Te=100ms


	fcntl(serveur,F_SETFL,fcntl(serveur,F_GETFL) | O_NONBLOCK);

	Eigen::VectorXd qf(6); qf << 0.0, 40.5, 35.0, 0.0, 59.5, 0.0;
    qf = qf * M_PI / 180.0;  // Convert to radians

    robot.simuTrapeze(0, 0, qf, 3.0, dt, VELOCITY);

	close(serveur);

	return 0;

}

/*
 * Communication with bloc retard
 */
void sendCmd(int clientID, int *handles, const Eigen::VectorXd& q, CmdType_t cmdType) {
	for (int i = 0; i < NB_JOINTS; i++) {
		message.q_cmd[i] = q(i);
	}
	message.cmdType = cmdType;
	gettimeofday(&message.time, NULL);

	int results, resultr;

	results=sendto(serveur,&message,sizeof(message),0,(struct sockaddr*)&sockAddr,sizeof(sockAddr));

	resultr=recvfrom(serveur,&message,sizeof(message), 0,(struct sockaddr*)&sockAddr,&addr);

	printf("--- client --- \n  rr=%d rs=%d\n time=%ld.%ld\n ",resultr, results, message.time.tv_sec,message.time.tv_usec);

	usleep(dt*1000000); // Sleep for the duration of the time step (replace syncrhonous mode in Coppelia)
}

void getJointPosition(int clientID, int jointHandle, double* position) {

}
