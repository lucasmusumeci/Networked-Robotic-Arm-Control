#
# Compiler configuration
#
#
USER_DEFINES=-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DDO_NOT_USE_SHARED_MEMORY
CC=gcc -O3 $(USER_DEFINES)

#
# Set VREP path for include files and 
# required C files in remoteApi folder:
#     extApi.c, extApiPlatform.c 
#
PATH_ROBMANIP = /home/lucas/Documents/Polytech/S8/Autom_Reseaux/robot
INCLUDE = -I$(PATH_ROBMANIP) -I/usr/include/eigen3

#
# Targets
#
#
EXE = client_robot
OBJS = robManip.o client_robot.o

all: 		$(OBJS)
			g++ $? -o $(EXE)
			
client_robot.o: 	client_robot.cpp
			$(CC) -c $(INCLUDE) client_robot.cpp
		
robManip.o: $(PATH_ROBMANIP)/robManip.hpp $(PATH_ROBMANIP)/robManip.cpp
			$(CC) -c $(INCLUDE) $(PATH_ROBMANIP)/robManip.cpp
			
clean:
			rm *.o $(EXE)
