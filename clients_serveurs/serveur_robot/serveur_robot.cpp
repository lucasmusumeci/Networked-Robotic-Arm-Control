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
	double q_cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	int cmdType;
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

int main (int nba, char *arg[])
{
    // Start the simulator and get handles
    int portNb=5555;            // the port number where to connect
    int timeOutInMs=5000;       // connection time-out in milliseconds (for the first connection)
    int commThreadCycleInMs=5;  // indicate how often data packets are sent back and forth - a default value of 5 is recommended

    // Connection to the server
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,timeOutInMs,commThreadCycleInMs);

    GetHandles(clientID);


	// Serveur (127.0.0.1)
	msg_t message;
	int result, nsend;
	struct sockaddr_in sockAddr, sock;
	int client, err, nConnect;
    socklen_t addr;
	int results, resultr ;
	long int  Te;

	client=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr.sin_family=PF_INET;
	sockAddr.sin_port=htons(2000);
	sockAddr.sin_addr.s_addr=inet_addr("127.0.0.1");
	addr=sizeof(sockAddr);

	err=bind(client,(struct sockaddr*)&sockAddr,addr);
	if(err==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	Te=100000; // Te=100ms

	results=ERROR;
	resultr=ERROR;

	fcntl(client,F_SETFL,fcntl(client,F_GETFL) | O_NONBLOCK);

	do
	{

		resultr=recvfrom(client,&message,sizeof(message), 0,(struct sockaddr*)&sockAddr,&addr);

		printf("--- server --- \n rt=%d rr=%d\n time=%ld.%ld\n",results,resultr,message.time.tv_sec,message.time.tv_usec);

        if (message.cmdType == 0)
        {
            // Implementation for sending position command
            for (int i = 0; i < NB_JOINTS; i++)
            {
                simxSetJointTargetPosition(clientID, handles[i], message.q_cmd[i], simx_opmode_oneshot);
            }
        }
        else if (message.cmdType == 1)
        {
            // Implementation for sending velocity command
            for (int i = 0; i < NB_JOINTS; i++)
            {
                simxSetJointTargetVelocity(clientID, handles[i], message.q_cmd[i], simx_opmode_oneshot);
            }
        }
        
        // Trigger a simulation step
        simxSynchronousTrigger(clientID);

		results=sendto(client,&message,sizeof(message),0,(struct sockaddr*)&sockAddr,sizeof(sockAddr));

	}while(1);

	usleep(Te);
	close(client);

	return 0;

}

