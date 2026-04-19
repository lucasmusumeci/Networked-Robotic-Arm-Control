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

#define NB_JOINTS 6
typedef struct {
	int cmdType;
	double cmd[NB_JOINTS];
	double q_simu[NB_JOINTS];
	double qdot_simu[NB_JOINTS];
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
#define DELAY_US 500000 // 200ms

int main (int nba, char *arg[])
{
    msg_delay_t tab_msg[SIZE_MSG_BUFFER] = {};
    
    //double normal = gsl_ran_gaussian(rng, 1.0);  // stddev=1.0
    //int poisson = gsl_ran_poisson(rng, 5.0);     // lambda=5.0

    // Client de 127.0.0.1
	msg_t message_client;
	int addr_client;
	struct sockaddr_in sockAddr_client;
	int client, results_client, resultr_client;

	client=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_client.sin_family=PF_INET;
	sockAddr_client.sin_port=htons(2001);
	sockAddr_client.sin_addr.s_addr=inet_addr("127.0.0.1");
	addr_client=sizeof(sockAddr_client);

	fcntl(client,F_SETFL,fcntl(client,F_GETFL) | O_NONBLOCK);

    // Serveur (127.0.0.2)
	msg_t message_serveur;
	int addr_serveur;
	struct sockaddr_in sockAddr_serveur;
	int serveur, results_serveur, resultr_serveur ;

	serveur=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_serveur.sin_family=PF_INET;
	sockAddr_serveur.sin_port=htons(2002);
	sockAddr_serveur.sin_addr.s_addr=inet_addr("127.0.0.2");
	addr_serveur=sizeof(sockAddr_serveur);

	int err_serveur=bind(serveur,(struct sockaddr*)&sockAddr_serveur,addr_serveur);
	if(err_serveur==ERROR)
	{
		printf("\n erreur de bind du serveur UDP!! \n");
	}

	results_serveur=ERROR;
	resultr_serveur=ERROR;

	fcntl(serveur,F_SETFL,fcntl(serveur,F_GETFL) | O_NONBLOCK);

	long int  Te=1; // Te=2ms
	int empty_slot = -1;

	do
	{
		usleep(Te);

        // Receive message from client
		resultr_serveur=recvfrom(serveur,&message_serveur,sizeof(message_serveur), 0,(struct sockaddr*)&sockAddr_serveur,&addr_serveur);

		if(resultr_serveur != ERROR) {
            struct timeval current_time;
            gettimeofday(&current_time, NULL);
            long int delay_us = (current_time.tv_sec - message_serveur.time.tv_sec) * 1000000 + (current_time.tv_usec - message_serveur.time.tv_usec);
            printf("--- server --- \n delay= %d ms\n ", delay_us/1000);
        }
        
		if(empty_slot == -1) empty_slot = find_empty_slot(tab_msg, SIZE_MSG_BUFFER);

        if (resultr_serveur != ERROR && empty_slot != -1)
        {
			// Random delay
			//int delay_ms = 20 + (random() % 981);
			// Constant delay
			int delay_ms = DELAY_US / 1000;
			tab_msg[empty_slot].msg = message_serveur;
            tab_msg[empty_slot].time_to_send.tv_sec  = message_serveur.time.tv_sec  + delay_ms*1000/1000000;
            tab_msg[empty_slot].time_to_send.tv_usec = message_serveur.time.tv_usec + (delay_ms*1000)%1000000; 
			empty_slot = -1;  
		}

		//printf("--- server --- \n rt=%d rr=%d\n time=%ld.%ld\n",results_serveur,resultr_serveur,message_serveur.time.tv_sec,message_serveur.time.tv_usec);

		//results_serveur=sendto(serveur,&message_serveur,sizeof(message_serveur),0,(struct sockaddr*)&sockAddr_serveur,sizeof(sockAddr_serveur));


        // Send message to server
        for(int i=0 ; i<SIZE_MSG_BUFFER ; i++)
        {
            if (tab_msg[i].time_to_send.tv_sec != 0 || tab_msg[i].time_to_send.tv_usec != 0)
            {
                struct timeval now;
                gettimeofday(&now, NULL);
                if (diff_time_us(&now, &tab_msg[i].time_to_send) >= 0)
                {
                    message_client = tab_msg[i].msg;
                    results_client=sendto(client,&message_client,sizeof(message_client),0,(struct sockaddr*)&sockAddr_client,sizeof(sockAddr_client));

                    // Clear the slot
                    tab_msg[i].time_to_send.tv_sec = 0;
                    tab_msg[i].time_to_send.tv_usec = 0;
					if (i < empty_slot) empty_slot = i;
                }
            }
			else if (i < empty_slot)
			{
				empty_slot = i;
			}
        }
		
		//resultr_client=recvfrom(client,&message_client,sizeof(message_client), 0,(struct sockaddr*)&sockAddr_client,&addr_client);

        //printf("--- client --- \n rr=%d rs=%d\n time=%ld.%ld\n ",resultr_client, results_client, message_client.time.tv_sec,message_client.time.tv_usec);

	}while(1);

	close(client);
	close(serveur);
	return 0;
}

