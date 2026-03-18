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

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#define NB_JOINTS 6
typedef struct {
	double q_cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	int cmdType;
	struct timeval time;
}msg_t;

typedef struct {
    msg_t msg;
    struct timeval time_to_send;
}msg_delay_t;

#define SIZE_MSG_BUFFER 100

int diff_time_us(struct timeval *t1, struct timeval *t2)
{
    return (t1->tv_sec - t2->tv_sec) * 1000000 + (t1->tv_usec - t2->tv_usec);
}

int find_empty_slot(msg_delay_t *buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (buffer[i].time_to_send.tv_sec == 0 && buffer[i].time_to_send.tv_usec == 0)
        {
            return i;
        }
    }
    return -1; // No empty slot found
}

#define ERROR (-1)
#define DELAY_US 500000 // 500ms

int main (int nba, char *arg[])
{
    msg_delay_t tab_msg[SIZE_MSG_BUFFER];
    gsl_rng *rng = gsl_rng_alloc(gsl_rng_mt19937);
    
    //double normal = gsl_ran_gaussian(rng, 1.0);  // stddev=1.0
    //int poisson = gsl_ran_poisson(rng, 5.0);     // lambda=5.0

    // Client (0)
	int result_client;
	int nsend_client;
	int nconnect_client;
	msg_t message_client;
	int addr_client;

	struct sockaddr_in sockAddr_client, sock_client;
	int client, err_client, nConnect_client, longaddr_client , results_client, resultr_client;

	client=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_client.sin_family=PF_INET;
	sockAddr_client.sin_port=htons(2000);
	sockAddr_client.sin_addr.s_addr=inet_addr("127.0.0.1");
	addr_client=sizeof(sockAddr_client);

	fcntl(client,F_SETFL,fcntl(client,F_GETFL) | O_NONBLOCK);

    // Serveur (1) (127.0.0.2)
	msg_t message_serveur;
	int result_serveur, nsend_serveur;
	struct sockaddr_in sockAddr_serveur, sock_serveur;
	int serveur, client_serveur, err_serveur, nConnect_serveur, longaddr_serveur;
	int results_serveur, resultr_serveur ;

	serveur=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_serveur.sin_family=PF_INET;
	sockAddr_serveur.sin_port=htons(2001);
	sockAddr_serveur.sin_addr.s_addr=inet_addr("127.0.0.2");
	longaddr_serveur=sizeof(sockAddr_serveur);

	err_serveur=bind(serveur,(struct sockaddr*)&sockAddr_serveur,longaddr_serveur);
	if(err_serveur==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	long int  Te=25000; // Te=25ms

	results_serveur=ERROR;
	resultr_serveur=ERROR;

	fcntl(serveur,F_SETFL,fcntl(serveur,F_GETFL) | O_NONBLOCK);

	do
	{
		usleep(Te);

        // Receive message from client 1
		resultr_serveur=recvfrom(serveur,&message_serveur,sizeof(message_serveur), 0,(struct sockaddr*)&sockAddr_serveur,&longaddr_serveur);
        int empty_slot = find_empty_slot(tab_msg, SIZE_MSG_BUFFER);
        if (resultr_serveur > 0 && empty_slot != -1)
        {
            gettimeofday(&tab_msg[empty_slot].time_to_send, NULL);

			// Random delay
			//int delay_ms = gsl_ran_poisson(rng, 500.0);

			int delay_us = DELAY_US;
            tab_msg[empty_slot].time_to_send.tv_sec += delay_us/1000000;
            tab_msg[empty_slot].time_to_send.tv_usec += (delay_us%1000000);
            tab_msg[empty_slot].msg = message_serveur;
        }

		printf("--- server --- \n rt=%d rr=%d\n time=%ld.%ld\n",results_serveur,resultr_serveur,message_serveur.time.tv_sec,message_serveur.time.tv_usec);

		results_serveur=sendto(serveur,&message_serveur,sizeof(message_serveur),0,(struct sockaddr*)&sockAddr_serveur,sizeof(sockAddr_serveur));


        // Send message to server 0
        for(int i=0 ; i<SIZE_MSG_BUFFER ; i++)
        {
            if (tab_msg[i].time_to_send.tv_sec != 0 || tab_msg[i].time_to_send.tv_usec != 0)
            {
                struct timeval now;
                gettimeofday(&now, NULL);
                if (diff_time_us(&now, &tab_msg[i].time_to_send) >= 0)
                {
                    message_client = tab_msg[i].msg;
                    gettimeofday(&message_client.time, NULL);
                    results_client=sendto(client,&message_client,sizeof(message_client),0,(struct sockaddr*)&sockAddr_client,sizeof(sockAddr_client));

                    // Clear the slot
                    tab_msg[i].time_to_send.tv_sec = 0;
                    tab_msg[i].time_to_send.tv_usec = 0;
                }
            }
        }
		
		resultr_client=recvfrom(client,&message_client,sizeof(message_client), 0,(struct sockaddr*)&sockAddr_client,&addr_client);

        printf("--- client --- \n rr=%d rs=%d\n time=%ld.%ld\n ",resultr_client, results_client, message_client.time.tv_sec,message_client.time.tv_usec);

	}while(1);

	usleep(Te);
    gsl_rng_free(rng);
	return 0;

}

