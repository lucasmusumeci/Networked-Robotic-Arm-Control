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

typedef struct {
	double label;
	double position[1];
	double control[1];
	struct timeval time;
}msg_t;

#define ERROR (-1)

int main (int nba, char *arg[])
{
	// Serveur (127.0.0.1)
	msg_t message;
	int result, nsend;
	struct sockaddr_in sockAddr, sock;
	int client, err, nConnect, longaddr;
	int results, resultr ;
	long int  Te;

	client=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr.sin_family=PF_INET;
	sockAddr.sin_port=htons(2000);
	sockAddr.sin_addr.s_addr=inet_addr("127.0.0.1");
	longaddr=sizeof(sockAddr);

	err=bind(client,(struct sockaddr*)&sockAddr,longaddr);
	if(err==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	message.label=0.0;
	message.position[0]=0.0;
	message.control[0]=0.0;


	Te=100000; // Te=100ms

	results=ERROR;
	resultr=ERROR;

	fcntl(client,F_SETFL,fcntl(client,F_GETFL) | O_NONBLOCK);

	do
	{
		usleep(Te);

		resultr=recvfrom(client,&message,sizeof(message), 0,(struct sockaddr*)&sockAddr,&longaddr);

		printf("--- server --- \n label=%lf rt=%d rr=%d\n time=%ld.%ld\n",message.label,results,resultr,message.time.tv_sec,message.time.tv_usec);

		results=sendto(client,&message,sizeof(message),0,(struct sockaddr*)&sockAddr,sizeof(sockAddr));

	}while(message.label<100.0);

	usleep(Te);
	close(client);

	return 0;

}

