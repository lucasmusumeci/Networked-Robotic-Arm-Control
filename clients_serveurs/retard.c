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

typedef struct {
	double label;
	double position[1];
	double control[1];
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
#define DELAY_S 0
#define DELAY_US 500000 // 500ms

int main (int nba, char *arg[])
{
    msg_delay_t tab_msg[SIZE_MSG_BUFFER];
    gsl_rng *r = gsl_rng_alloc(gsl_rng_mt19937);
    
    //double normal = gsl_ran_gaussian(r, 1.0);  // stddev=1.0
    //int poisson = gsl_ran_poisson(r, 5.0);     // lambda=5.0

    // Client (0)
	int result_0;
	int nsend_0;
	int nconnect_0;
	msg_t message_0;
	int addr_0;

	struct sockaddr_in sockAddr_0, sock_0;
	int client_0, err_0, nConnect_0, longaddr_0 , results_0, resultr_0;

	client_0=socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP);
	sockAddr_0.sin_family=PF_INET;
	sockAddr_0.sin_port=htons(2000);
	sockAddr_0.sin_addr.s_addr=0;
	addr_0=sizeof(sockAddr_0);

	message_0.label=0.0;
	message_0.position[0]=0.0;
	message_0.control[0]=0;

	fcntl(client_0,F_SETFL,fcntl(client_0,F_GETFL) | O_NONBLOCK);

    // Serveur (1)
	msg_t message_1;
	int result_1, nsend_1;
	struct sockaddr_in sockAddr_1, sock_1;
	int serveur_1, client_1, err_1, nConnect_1, longaddr_1;
	int results_1, resultr_1 ;

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


	long int  Te=100000; // Te=100ms

	results_1=ERROR;
	resultr_1=ERROR;

	fcntl(serveur_1,F_SETFL,fcntl(serveur_1,F_GETFL) | O_NONBLOCK);

	do
	{
		usleep(Te);

        // Receive message from client_1
		resultr_1=recvfrom(serveur_1,&message_1,sizeof(message_1), 0,(struct sockaddr*)&sockAddr_1,&longaddr_1);
        int empty_slot = find_empty_slot(tab_msg, SIZE_MSG_BUFFER);
        if (resultr_1 > 0 && empty_slot != -1)
        {
            gettimeofday(&tab_msg[empty_slot].time_to_send, NULL);
            tab_msg[empty_slot].time_to_send.tv_sec += DELAY_S;
            tab_msg[empty_slot].time_to_send.tv_usec += DELAY_US;
            tab_msg[empty_slot].msg = message_1;
        }

		printf(" server : \n label=%lf rt=%d rr=%d\n time=%ld.%ld\n",message_1.label,results_1,resultr_1,message_1.time.tv_sec,message_1.time.tv_usec);

		results_1=sendto(serveur_1,&message_1,sizeof(message_1),0,(struct sockaddr*)&sockAddr_1,sizeof(sockAddr_1));


        // Send message to server_0
        for(int i=0 ; i<SIZE_MSG_BUFFER ; i++)
        {
            if (tab_msg[i].time_to_send.tv_sec != 0 || tab_msg[i].time_to_send.tv_usec != 0)
            {
                struct timeval now;
                gettimeofday(&now, NULL);
                if (diff_time_us(&now, &tab_msg[i].time_to_send) >= 0)
                {
                    message_0 = tab_msg[i].msg;
                    gettimeofday(&message_0.time, NULL);
                    results_0=sendto(client_0,&message_0,sizeof(message_0),0,(struct sockaddr*)&sockAddr_0,sizeof(sockAddr_0));
                    resultr_0=recvfrom(client_0,&message_0,sizeof(message_0), 0,(struct sockaddr*)&sockAddr_0,&addr_0);

                    printf("\n client : \n  label=%lf rr=%d rs=%d\n time=%ld.%ld\n ",message_0.label,resultr_0, results_0, message_0.time.tv_sec,message_0.time.tv_usec);

                    // Clear the slot
                    tab_msg[i].time_to_send.tv_sec = 0;
                    tab_msg[i].time_to_send.tv_usec = 0;
                }
            }
        }

	}while(message_0.label<100.0);

    gsl_rng_free(r);
	return 0;

}

