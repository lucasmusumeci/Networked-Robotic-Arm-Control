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
PATH_VREP=/opt/CoppeliaSim
INCLUDE1 = $(PATH_VREP)/programming/include
INCLUDE2 = $(PATH_VREP)/programming/legacyRemoteApi/remoteApi
INCLUDE = -I$(INCLUDE1) -I$(INCLUDE2) -I$(PATH_ROBMANIP) -I/usr/include/eigen3

#
# Targets
#
#
EXE = step_ident
OBJS = robManip.o extApi.o extApiPlatform.o step_ident.o

all: 		$(OBJS)
			g++ $? -o $(EXE)
			
step_ident.o: 	step_ident.cpp
			$(CC) -c $(INCLUDE) step_ident.cpp
		
robManip.o: $(PATH_ROBMANIP)/robManip.hpp $(PATH_ROBMANIP)/robManip.cpp
			$(CC) -c $(INCLUDE) $(PATH_ROBMANIP)/robManip.cpp

extApi.o: 	$(INCLUDE2)/extApi.c
			$(CC) -c $(INCLUDE) $(INCLUDE2)/extApi.c
			
extApiPlatform.o:	$(INCLUDE2)/extApiPlatform.c
			$(CC) -c $(INCLUDE) $(INCLUDE2)/extApiPlatform.c
			
clean:
			rm *.o $(EXE)
