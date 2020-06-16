#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <math.h>

#include "robot.h"


/************* UTILITY FUNCTIONS ****************/

float limiter(float X)
{
	//function returns -1 if x < -1, 0 if x==0, 1 if x > 1.
	float temp=X;
	if(temp<0)
		temp=temp*-1;
	if(temp>1){
		if(X<0)
			return -1;
		else if(X==0)
			return 0;
		else if(X>0)
			return 1;
	}
	else
		return X;
}



/****************************************************/
/****************************************************/



/******************* ROBOTJOINT ****************************/

RobotJoint::RobotJoint(){
	pos = 0;
	id = 0;
	ref = 0;
}

uint16_t RobotJoint::ICSPos(float offset){
	return (uint16_t) (ref + mult_factor*SLOPE*((pos + offset)*180.0/3.14));
}

uint16_t RobotJoint::ICSPos(){
	ICSPos(0);
}

int RobotJoint::getPos(uint8_t *data, float offset){

	uint16_t icsPos;
	icsPos = ICSPos(offset);
	data[0] = id;
	data[1] = (icsPos & 0xFF00) >> 8;
	data[2] = (icsPos & 0xFF);

	return 3;
}

int RobotJoint::getPos(uint8_t *data){
	getPos(data, 0);
}


void RobotJoint::printPos(){
	printf("%f", pos);
}

/***************************************************/
/***************************************************/

/************************* ROBOT LEG *************************/

// Function to determine the inverse kinematics of the stoch robot leg.
// Accepts task-space inputs (in meters) and returns all the joint angles
// 
void RobotLeg::inverseKinematics(float x, float y, float *hip_angle, float *knee_angle){
	float q1,q2,q3,q4;

	float xb[] = {0,0};
	float yb[] = {0,0};
	float phid[] = {0,0};
	float psi[] = {0,0};
	float theta[] = {0,0};
	float R_base[]= {0,0.035,0,0};
	float zeta,xm,ym,xi,yi;
	float R[] = {0,0};

	float l1 =   0.12;
	float l2 =   0.15015;
	float l4 =   0.04;
	float l5 =   0.15501;
	float le =   0.04;
	float tq1 =  0.2532;
	float tq2 =  2.803;
	float delta = 0.11187;

	float l3,l6;

	xb[0] = R_base[0];
	xb[1] = R_base[1];
	yb[0] = R_base[2];
	yb[1] = R_base[3];


	l3 = ((x-xb[0])*(x-xb[0])+(y-yb[0])*(y-yb[0]));
	l3=sqrtf(l3);
	theta[0] = atan2f((y-yb[0]),(x-xb[0]));
	zeta = (powf(l3,2) - powf(l1,2) -powf(l2,2))/(2*l1*l2);
	zeta = limiter(zeta);
	phid[0] = acosf(zeta);
	psi[0] = atan2f(l2*sinf(phid[0]),(l1+l2*cosf(phid[0])));
	q1 = theta[0] - psi[0];
	q2 = q1 + phid[0];


	xi = xb[0] + l1*cosf(q1) + 0.04*cosf(q2-tq1);
	yi = yb[0] + l1*sinf(q1) + 0.04*sinf(q2-tq1);
	l6=(xi-xb[1])*(xi-xb[1])+(yi-yb[1])*(yi-yb[1]);
	l6=sqrtf(l6);
	theta[1] = atan2f((yi-yb[1]),(xi-xb[1]));
	zeta=(powf(l6,2) - powf(l4,2) -powf(l5,2))/(2*l5*l4);
	zeta = limiter(zeta);
	phid[1] = acosf(zeta);
	psi[1] = atan2f(l5*sinf(phid[1]),(l4+l5*cosf(phid[1])));
	q3 = theta[1] + psi[1];
	q4 = q3 - phid[1];
	xm = l4*cosf(q3)+l5*cosf(q4)+xb[1];
	ym = l4*sinf(q3)+l5*sinf(q4)+yb[1];
	if (zeta == 1)
	{
		l1 = 0.12;
		l2 = 0.03828;//Leg[1]-Leg[4];
		l4 = 0.04;
		l5 = 0.15501;
		delta = 0.11187;
		R_base[0]= 1;
		R_base[1]=-1;
		R_base[2]=0;
		R_base[3]=0;
		xb[0] = R_base[0];
		xb[1] = R_base[1];
		yb[0] = R_base[2];
		yb[1] = R_base[3];
		l3 = sqrtf(powf((xm-xb[0]),2)+powf((ym-yb[0]),2));
		theta[0] = atan2f((ym-yb[0]),(xm-xb[0]));
		zeta = (powf(l3,2) - powf(l1,2) -powf(l2,2))/(2*l1*l2);
		zeta = limiter(zeta);
		phid[0] = acosf(zeta);
		psi[0] = atan2f(l2*sinf(phid[0]),(l1+l2*cosf(phid[0])));
		q1 = theta[0] + psi[0];
		q2 = q1 - phid[0];

	}
	*hip_angle  = q1;
	*knee_angle = q3;
}


void RobotLeg::inverseKinematics3D(float x, float y, float z, float *hip, float *knee, float *abd){

	float joint_offset = 0.03; // Distance between the axis of rotation of the abduction and hip/knee

	*abd = atan2(z, -y);
	inverseKinematics(x, y/cos(*abd) + joint_offset, hip, knee);

	return;
}


bool RobotLeg::validData(float x, float y, float z){

	float abdAngle;

	if(y>-0.145 || y<-0.245)
	{
		if ( (y > -0.145) & (x > 0.0249) & (x < 0.0251)){
			// Condition required to keep the stand sequence valid.
			return true;
		}else{
			return false;
		}
	}
	else if(x>(-1*(y+0.01276)/(1.9737)) || x<((y+0.01276)/(1.9737)))
	{
		return false;
	}

	abdAngle = atan2(z, -y);
	abdAngle = abdAngle * 180.0/3.14; //deg

	// Limit the abdAngle
	if (abdAngle < -10) // -10 deg
		return false;
	else if (abdAngle > 45) // 45 deg
		return false;

	return true;

}


bool RobotLeg::toJointSpace(float x, float y, float z){

	if (validData(x, y, z) == false)
		return false;

	inverseKinematics3D(x, y, z, &hip.pos, &knee.pos, &abd.pos);

	return true;

}

int RobotLeg::getPos(uint8_t *data){

	int len= 0;
	int ret = 0;

	ret = hip.getPos(&data[len], 3.14);
	len += ret;

	ret = knee.getPos(&data[len]);
	len += ret;

	ret = abd.getPos(&data[len]);
	len += ret;

	return len;
}


void RobotLeg::printPos(){
	hip.printPos();
	printf(", ");
	knee.printPos();
	printf(", ");
	abd.printPos();
	printf("\n");
}

/*************************************************************/
/*************************************************************/



/***************** STOCHROBOT *******************************/

StochRobot::StochRobot(){

	leg[FL].id = FL_ID; 
	leg[FL].hip.id  = FL_HIP_ID;
	leg[FL].knee.id = FL_KNEE_ID;
	leg[FL].abd.id  = FL_ABD_ID;

	leg[FL].hip.ref  = FL_HIP_REF;
	leg[FL].knee.ref = FL_KNEE_REF;
	leg[FL].abd.ref  = FL_ABD_REF;

	leg[FL].hip.mult_factor  = 1;
	leg[FL].knee.mult_factor = 1;
	leg[FL].abd.mult_factor  = 1;

	leg[FR].id = FR_ID; 
	leg[FR].hip.id  = FR_HIP_ID;
	leg[FR].knee.id = FR_KNEE_ID;
	leg[FR].abd.id  = FR_ABD_ID;

	leg[FR].hip.ref  = FR_HIP_REF;
	leg[FR].knee.ref = FR_KNEE_REF;
	leg[FR].abd.ref  = FR_ABD_REF;

	leg[FR].hip.mult_factor  = -1;
	leg[FR].knee.mult_factor = -1;
	leg[FR].abd.mult_factor  = -1;

	leg[BL].id = BL_ID; 
	leg[BL].hip.id  = BL_HIP_ID;
	leg[BL].knee.id = BL_KNEE_ID;
	leg[BL].abd.id  = BL_ABD_ID;

	leg[BL].hip.ref  = BL_HIP_REF;
	leg[BL].knee.ref = BL_KNEE_REF;
	leg[BL].abd.ref  = BL_ABD_REF;

	leg[BL].hip.mult_factor  = 1;
	leg[BL].knee.mult_factor = 1;
	leg[BL].abd.mult_factor  = 1;

	leg[BR].id = BR_ID; 
	leg[BR].hip.id  = BR_HIP_ID;
	leg[BR].knee.id = BR_KNEE_ID;
	leg[BR].abd.id  = BR_ABD_ID;

	leg[BR].hip.ref  = BR_HIP_REF;
	leg[BR].knee.ref = BR_KNEE_REF;
	leg[BR].abd.ref  = BR_ABD_REF;

	leg[BR].hip.mult_factor  = -1;
	leg[BR].knee.mult_factor = -1;
	leg[BR].abd.mult_factor  = -1;

}

StochRobot::~StochRobot(){
}

int StochRobot::getPos(uint8_t *data){
	int len=0;
	int ret=0;
	int i=0;
	for(i=0; i<4; i++){
		ret = leg[i].getPos(&data[len]);
		len += ret;
	}

	return len;

}

/*************************************************************/
/*************************************************************/
