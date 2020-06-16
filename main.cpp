#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <math.h>

#include "walking_controller.h"
#include "robot.h"
#include "comm.h"

#define CMD_SET_POS 1


void printUtil(uint8_t *data, uint8_t data_len){
	int i=0;
	uint8_t posLo, posHi;
	uint16_t pos;
	for(i=0; i<data_len/3;i++){
		posHi = data[3*i + 1];
		posLo = data[3*i + 2];
		pos = (posHi << 8) + posLo;
		printf("%d, ",pos);
	}
	printf("\n");
}




void standSequence(StochRobot robot, StochComm comm){

	uint8_t data[256] = {0};
	uint8_t data_len=0;
	float x, y;

	for(int i=0; i < 100; i++) {
		x = 0.025;
    		y = -0.03  -0.1201 + 0.07485*cos(i*3.14/100.0);

		//printf("x: %f, y: %f\n", x, y);
		robot.leg[FL].toJointSpace(x, y, 0);
		robot.leg[FR].toJointSpace(x, y, 0);
		robot.leg[BL].toJointSpace(x, y, 0);
		robot.leg[BR].toJointSpace(x, y, 0);

		data_len = robot.getPos(data);
		printf("%d: ",i);
		printUtil(data, data_len);
		//comm.sendData(CMD_SET_POS, data, data_len);
	}

 	return;
}



int main(int argc, char **argv){
	
	StochComm comm(true);
	StochRobot robot;

	int ret;

	uint8_t data[256];
	uint8_t data_recv[256];
	uint8_t data_len=0;
	uint8_t i=0;

	ret = comm.init();
	if (ret == -1) {
		printf("Error: Unable to initialize communications module.\n");
		return -1;
	}

	// Phase for trot-gait
	int phase[]={0, 100, 100, 0}; // FL FR BL BR 
	WalkingController::WalkingController walkcon(phase);

	// Use a fixed-action for testing
	float action[]={0.068, 0.068, 0.068, 0.068, 0, 0, 0, 0};
	for(i = 0; i < 200; i++){
		// Obtain the foot positions from the walking controller
		walkcon.runEllipticTrajectory(i, action, 1000);
		
		// Convert the foot positions to joint-angles
		robot.leg[FL].toJointSpace(walkcon.leg[FL].x, walkcon.leg[FL].y, walkcon.leg[FL].z);
		robot.leg[FR].toJointSpace(walkcon.leg[FR].x, walkcon.leg[FR].y, walkcon.leg[FR].z);
		robot.leg[BL].toJointSpace(walkcon.leg[BL].x, walkcon.leg[BL].y, walkcon.leg[BL].z);
		robot.leg[BR].toJointSpace(walkcon.leg[BR].x, walkcon.leg[BR].y, walkcon.leg[BR].z);

		// Obtain the joint-angle data that needs to be sent to the motors
		data_len = robot.getPos(data);
		printUtil(data, data_len);

		// Send the joint-angle data to the Tiva board which will in-turn send it
		// to each of the motors.
		//comm.sendData(CMD_SET_POS, data, data_len);
	}
	return 0;
}


