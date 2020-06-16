#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "walking_controller.h"

namespace WalkingController {


	RobotLeg::RobotLeg(){
		// Initialize all the variables
		phase = 0;
		plane_angle = 0;
		x = 0;
		y = 0;
		z = 0;

	}

	void RobotLeg::addPhase(int ph){

		phase = ph;

		// Ensure phase lies in [0, max_phase) range
		while (phase < 0)
			phase += max_phase;
		phase = phase % max_phase;

		return;
	}






	WalkingController::WalkingController(int *leg_phase){

		gait_phase[FL] = leg_phase[FL];
		gait_phase[FR] = leg_phase[FR];
		gait_phase[BL] = leg_phase[BL];
		gait_phase[BR] = leg_phase[BR];


		body_length = 0.37;
		body_width  = 0.24;

	}


	void WalkingController::runEllipticTrajectory(int phase, float *action, float radius) {

		const float	step_length = 0.068;
		float 		new_radius;
		float 		leg_theta;
		float 		leg_r;
		const float 	y_center = -0.243;
		float 		flag;
		float 		x;
		float 		y;

		// Phase 
		leg[FL].addPhase(phase + gait_phase[FL]);
		leg[FR].addPhase(phase + gait_phase[FR]);
		leg[BL].addPhase(phase + gait_phase[BL]);
		leg[BR].addPhase(phase + gait_phase[BR]);

		// Plane angle
		if (radius >= 0.0){
			leg[FL].plane_angle =  atan2(body_length/2, radius + body_width/2);
			leg[FR].plane_angle = -atan2(body_length/2, radius - body_width/2);
			leg[BL].plane_angle = -atan2(body_length/2, radius + body_width/2);
			leg[BR].plane_angle =  atan2(body_length/2, radius - body_width/2);
		} else if (radius < 0.0) {
			new_radius = -1*radius;
			leg[FL].plane_angle = -atan2(body_length/2, new_radius - body_width/2);
			leg[FR].plane_angle =  atan2(body_length/2, new_radius + body_width/2);
			leg[BL].plane_angle =  atan2(body_length/2, new_radius + body_width/2);
			leg[BR].plane_angle = -atan2(body_length/2, new_radius + body_width/2);		
		}


		// Step length
		if (fabs(radius) < 0.12){
			leg[FL].step_length = step_length;
			leg[FR].step_length = step_length;
			leg[BL].step_length = step_length;
			leg[BR].step_length = step_length;
		} else if (radius > 0.12) {
			leg[FL].step_length = step_length * (radius - body_width/2)/radius;
			leg[FR].step_length = step_length * (radius + body_width/2)/radius;
			leg[BL].step_length = step_length * (radius - body_width/2)/radius;
			leg[BR].step_length = step_length * (radius + body_width/2)/radius;
		} else if (radius < -0.12) {
			new_radius = -1*radius;
			leg[FL].step_length = step_length * (new_radius - body_width/2)/new_radius;
			leg[FR].step_length = step_length * (new_radius + body_width/2)/new_radius;
			leg[BL].step_length = step_length * (new_radius - body_width/2)/new_radius;
			leg[BR].step_length = step_length * (new_radius + body_width/2)/new_radius;
		}


		// Determine the foot positions
		for(int i=0; i < 3; i++){
			leg_theta = (((float) leg[i].phase)/leg[i].max_phase)*2*3.1415;
			leg_r = leg[i].step_length/2;
			x = -leg_r * cos(leg_theta);
			flag = leg_theta > 3.1415 ? 0 : 1;
			y = 0.06 * sin(leg_theta)*flag + y_center;

			leg[i].x = cos(leg[i].plane_angle)*x;
			leg[i].y = y;
			leg[i].z = -sin(leg[i].plane_angle)*x;

		}	

		return;

	}


}
