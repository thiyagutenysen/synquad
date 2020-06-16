#ifndef __WALKING_CONTROLLER_H__
#define __WALKING_CONTROLLER_H__

namespace WalkingController{

	class RobotLeg{
		public:
			int 	phase; // phase of the leg within one cycle
			int 	plane_angle; // plane angle of the leg
			float 	step_length;
			float 	x, y, z; // position co-ordinates of the foot in the leg frame
			RobotLeg();
			void addPhase(int ph);

			// Maximum value of phase variable;
			const int max_phase=200;
	};


	// Numbering starts from zero as arrays in C begin with an index of 0
#define FL 0
#define FR 1
#define BL 2
#define BR 3

	class WalkingController{
		public:
			int gait_phase[4]; // phase difference between the legs

			RobotLeg leg[4];

			WalkingController(int *leg_phase);

			void runEllipticTrajectory(int phase, float *action, float radius);
		private:
			float body_length;
			float body_width;
	};


}


#endif // __WALKING_CONTROLLER_H__
