#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "config.h"

#define SLOPE (8000/270)

#define FL_ID 0
#define FR_ID 1
#define BL_ID 2
#define BR_ID 3

class RobotJoint {
	public:
		RobotJoint();

		float pos; // Joint position in radians
		int id; // ID of the motor
		int ref; // Reference value for zero of the motor
		int mult_factor; // Multiplication factor, {1, -1} for calulating the command to send to the motor.

		uint16_t ICSPos(float offset);
		uint16_t ICSPos();

		int getPos(uint8_t *data, float offset);
		int getPos(uint8_t *data);

		void printPos();
};		

class RobotLeg {
	public:
		int id;
		RobotJoint hip;
		RobotJoint knee;
		RobotJoint abd;

		bool toJointSpace(float x, float y, float z);
		int getPos(uint8_t *data);
		void printPos();

	private:
		void inverseKinematics(float x, float y, float *hip, float *knee);
		void inverseKinematics3D(float x, float y, float z, float *hip, float *knee, float *abd);
		bool validData(float x, float y, float z);
};


#define FL 0
#define FR 1
#define BL 2
#define BR 3

class StochRobot {
	public:
		RobotLeg leg[4]; // FL FR BL BR

		StochRobot();
		~StochRobot();

		int getPos(uint8_t *data);
};

#endif // __ROBOT_H__
