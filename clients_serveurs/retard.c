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

struct mesg {
	double label;
	double position[1];
	double control[1];
	struct timeval time;
};

#define ERROR (-1)

int main (int nba, char *arg[])
{
    // Client (0)
	int result_0;
	int nsend_0;
	int nconnect_0;
	struct mesg message_0;
	int addr_0;
	long int Te_0;

	struct sockaddr_in sockAddr_0, sock_0;
	int client_0, err_0, nConnect_0, longaddr_0 , results_0, resultr_0;

	// Check if server address argument is provided
    if (nba < 2) {
        fprintf(stderr, "Usage: %s <server_ip>\n", arg[0]);
        exit(1);
    }

	client_0=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_0.sin_family=PF_INET;
	sockAddr_0.sin_port=htons(2000);
	sockAddr_0.sin_addr.s_addr=0;
	addr_0=sizeof(sockAddr_0);

	message_0.label=0.0;
	message_0.position[0]=0.0;
	message_0.control[0]=0;
	Te_0=100000; // Te=100ms

	fcntl(client_0,F_SETFL,fcntl(client_0,F_GETFL) | O_NONBLOCK);

    // Serveur (1)
	struct mesg message_1;
	int result_1, nsend_1;
	struct sockaddr_in sockAddr_1, sock_1;
	int serveur_1, client_1, err_1, nConnect_1, longaddr_1;
	int results_1, resultr_1 ;
	long int  Te_1;

	serveur_1=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_1.sin_family=PF_INET;
	sockAddr_1.sin_port=htons(2001);
	sockAddr_1.sin_addr.s_addr=0;
	longaddr_1=sizeof(sockAddr_1);

	err_1=bind(serveur_1,(struct sockaddr*)&sockAddr_1,longaddr_1);
	if(err_1==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	message_1.label=0.0;
	message_1.position[0]=0.0;
	message_1.control[0]=0.0;


	Te_1=100000; // Te=100ms

	results_1=ERROR;
	resultr_1=ERROR;

	fcntl(serveur_1,F_SETFL,fcntl(serveur_1,F_GETFL) | O_NONBLOCK);

    int i=0;
	do
	{
		usleep(Te_0);

        // Receive message from client_1
		resultr_1=recvfrom(serveur_1,&message_1,sizeof(message_1), 0,(struct sockaddr*)&sockAddr_1,&longaddr_1);

		printf("server : \n label=%lf rt=%d rr=%d\n time=%ld.%ld\n",message_1.label,results_1,resultr_1,message_1.time.tv_sec,message_1.time.tv_usec);

		results_1=sendto(serveur_1,&message_1,sizeof(message_1),0,(struct sockaddr*)&sockAddr_1,sizeof(sockAddr_1));


        // Send message to server_0
        message_0.label = message_1.label;
		gettimeofday(&message_0.time, NULL);
		results_0=sendto(client_0,&message_0,sizeof(message_0),0,(struct sockaddr*)&sockAddr_0,sizeof(sockAddr_0));
		resultr_0=recvfrom(client_0,&message_0,sizeof(message_0), 0,(struct sockaddr*)&sockAddr_0,&addr_0);

		printf("\n client : \n  label=%lf rr=%d rs=%d\n time=%ld.%ld\n ",message_0.label,resultr_0, results_0, message_0.time.tv_sec,message_0.time.tv_usec);

        i++;

	}while(message_0.label<100.0);

	return 0;

}

