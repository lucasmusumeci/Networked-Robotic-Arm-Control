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

using namespace std;

extern "C" {
    #include "extApi.h"
}

#define NB_JOINTS 6
typedef struct {
    int cmdType;
	double cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
    double qdot_simu[NB_JOINTS];
	struct timeval time;
}msg_t;

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

int main (int nba, char *arg[])
{
    // Start the simulator and get handles
    int portNb=5555;            // the port number where to connect
    int timeOutInMs=5000;       // connection time-out in milliseconds (for the first connection)
    int commThreadCycleInMs=5;  // indicate how often data packets are sent back and forth - a default value of 5 is recommended

    // Connection to the server
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,timeOutInMs,commThreadCycleInMs);

    if (clientID==-1)
    {
        printf("Connection to the server not possible\n");
        return(-1);
    }

    GetHandles(clientID);

    // Initialize streaming for joint positions (no blocking)
    for (int i = 0; i < 6; i++) {
        simxGetJointPosition(clientID, handles[i], nullptr, simx_opmode_streaming);
    }

    // Client de 127.0.0.3
    int client;
	socklen_t addr_client;
	struct sockaddr_in sockAddr_client;

	client=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_client.sin_family=PF_INET;
	sockAddr_client.sin_port=htons(2003);
	sockAddr_client.sin_addr.s_addr=inet_addr("127.0.0.3");
	addr_client=sizeof(sockAddr_client);

	fcntl(client,F_SETFL,fcntl(client,F_GETFL) | O_NONBLOCK);


	// Serveur (127.0.0.1)
    int serveur;
    socklen_t addr_serveur;
	struct sockaddr_in sockAddr_serveur;

	serveur=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_serveur.sin_family=PF_INET;
	sockAddr_serveur.sin_port=htons(2001);
	sockAddr_serveur.sin_addr.s_addr=inet_addr("127.0.0.1");
	addr_serveur=sizeof(sockAddr_serveur);

	int err_serveur = bind(serveur,(struct sockaddr*)&sockAddr_serveur,addr_serveur);
	if(err_serveur==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	fcntl(serveur,F_SETFL,fcntl(serveur,F_GETFL) | O_NONBLOCK);

    msg_t message = {};
    gettimeofday(&message.time, NULL);  // Initialize with current time
    int results, resultr;
    results=ERROR;
	

	do {
        resultr=ERROR; // Reset receive result for this iteration
        msg_t tmp; // Temporary variable to store received message

        // Read messages until we get the latest one (non-blocking)
        while(recvfrom(serveur,&tmp,sizeof(tmp), 0,(struct sockaddr*)&sockAddr_serveur,&addr_serveur) != ERROR)
        {
            resultr = 0; // Received at least 1 message
            // Only update if this message is newer than the latest one we have from the retard server
            if(diffTime_ms(&message.time, &tmp.time ) > 0) { // Compare timestamps to ensure we get the latest message
                message = tmp; // Update to the latest message
            }
        }

        if(resultr != ERROR) {
            struct timeval current_time;
            gettimeofday(&current_time, NULL);
            long int delay_us = (current_time.tv_sec - message.time.tv_sec) * 1000000 + (current_time.tv_usec - message.time.tv_usec);
            printf("--- server --- \n delay= %ld ms\n ", delay_us/1000);
        }
		//printf("--- server --- \n rt=%d rr=%d\n time=%ld.%ld\n",results,resultr,message.time.tv_sec,message.time.tv_usec);

        if (message.cmdType == 0)
        {
            // Implementation for sending position command
            for (int i = 0; i < NB_JOINTS; i++)
            {
                simxSetJointTargetPosition(clientID, handles[i], message.cmd[i], simx_opmode_oneshot);
            }
        }
        else if (message.cmdType == 1)
        {
            // Implementation for sending velocity command
            for (int i = 0; i < NB_JOINTS; i++)
            {
                simxSetJointTargetVelocity(clientID, handles[i], message.cmd[i], simx_opmode_oneshot);
            }
        }
        
        // Trigger a simulation step
        simxSynchronousTrigger(clientID); // Since realtime factor < 1, it will wait more than actual dt

        int all_positions_ok = 1;
        for (int i = 0; i < 6; i++)
        {
            simxFloat mesured_q;
            if (simxGetJointPosition(clientID, handles[i], &mesured_q, simx_opmode_buffer) == simx_return_ok)
            {
                message.q_simu[i] = static_cast<double>(mesured_q);
            }
            else
            {
                printf("Error reading position for joint %d\n", i);
                all_positions_ok = 0;
            }
        }

        if (all_positions_ok)
        {
            /*
            printf("theta = [");
            for (int i = 0; i < 6; ++i) {
                printf("%f", message.q_simu[i]);
                if (i < 5) printf(", ");
            }
            printf("]\n");
            */

            results=sendto(client,&message,sizeof(message),0,(struct sockaddr*)&sockAddr_client,sizeof(sockAddr_client));
        }
        else
        {
            printf("Skipping send due to position read errors\n");
        }

	}while(1);

	close(client);
    close(serveur);

    simxStopSimulation(clientID, simx_opmode_oneshot);

    // Close the connection to the server
    simxFinish(clientID);

	return 0;

}

