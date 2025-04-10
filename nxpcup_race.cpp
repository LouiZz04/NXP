#include "nxpcup_race.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

double printed_value = 1001;

uint8_t p_vec1_x0 = 0;
uint8_t p_vec1_y0 = 0;
uint8_t p_vec1_x1 = 0;
uint8_t p_vec1_y1 = 0;

uint8_t p_vec2_x0 = 0;
uint8_t p_vec2_y0 = 0;
uint8_t p_vec2_x1 = 0;
uint8_t p_vec2_y1 = 0;

uint8_t get_num_vectors(Vector &vec1, Vector &vec2) {
	uint8_t numVectors = 0;
	if(!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) numVectors++;
	if(!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) numVectors++;
	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num) {
	Vector vec;
	if(num == 1) {
		vec.m_x0 = pixy.m0_x0;
		vec.m_x1 = pixy.m0_x1;
		vec.m_y0 = pixy.m0_y0;
		vec.m_y1 = pixy.m0_y1;
	}
	if(num == 2) {
		vec.m_x0 = pixy.m1_x0;
		vec.m_x1 = pixy.m1_x1;
		vec.m_y0 = pixy.m1_y0;
		vec.m_y1 = pixy.m1_y1;
	}
	return vec;
}

float track_distance(Vector v1, Vector v2){
        return fabs(v1.m_x0 - v2.m_x1);
}

roverControl raceTrack(const pixy_vector_s &pixy)
{
	Vector main_vec;
	Vector vec1 = copy_vectors(pixy, 1);
	Vector vec2 = copy_vectors(pixy, 2);
	uint8_t frameWidth = 79;
	int16_t window_center = (frameWidth / 2);
	roverControl control{};
	//float x, y;					 // calc gradient and position of main vector
	static hrt_abstime no_line_time = 0;		// time variable for time since no line detected
	hrt_abstime time_diff = 0;
	static bool first_call = true;

	static hrt_abstime old_t = hrt_absolute_time();
	hrt_abstime t = hrt_absolute_time();
	double diff = difftime(t, old_t);
	// milliseconds
	diff = diff / 1000;

	uint8_t num_vectors = get_num_vectors(vec1, vec2);


	// new variables
	static float track_distance_unitless=75.f; // if haven't calculated it yet, but seeing 1 vector, assume 75, since pixy2 sees up to 79

        if(num_vectors == 2){
                track_distance_unitless = track_distance(vec1, vec2);
        }

	        // Stanley - Geometric Path tracking controller

        float steering_angle; // radians, thats my control.steer too.
        float heading_error; // radians, angle between can's direction and the desired path
        float cross_track_error; // meters, perpendicular distance between car and desired path
        //loat velocity; // m/s, which is my control.speed
        float k1_gain=0.3f, k2_gain=0.0f;

        //formula : steering_angle = heading_error + arctan( (k1_gain * cross_track_error / (velocity + 0.1)) + k2_gain * heading_error)



	switch (num_vectors) {
	case 0:
		if(first_call){
			no_line_time = hrt_absolute_time();
			first_call = false;
		}else{
			time_diff = hrt_elapsed_time_atomic(&no_line_time);
			control.steer = 0.0f;
			control.speed = 0.7f;
			if(time_diff > 100000*3){
				// Stopping if no vector is available
				control.speed = SPEED_STOP;
			}
		}
		break;

	case 2:{
		first_call = true;

		main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
		main_vec.m_x0 = (vec1.m_x0 + vec2.m_x0) / 2;
		main_vec.m_y1 = (vec1.m_y1 + vec2.m_y1) / 2;
		main_vec.m_y0 = (vec1.m_y0 + vec2.m_y0) / 2;

		//control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;


		// trial1

		float slope = (float(main_vec.m_x1-main_vec.m_x0))/(float(main_vec.m_y1-main_vec.m_y0));
		heading_error = float(atan(slope));

		float cross_track_error_inpixels = window_center - main_vec.m_x0;
		cross_track_error = 0.55f * cross_track_error_inpixels / track_distance_unitless; // 0.55m is my actually width of the track

		steering_angle = heading_error - float(atan( (float(k1_gain) * cross_track_error / (control.speed + 0.1f)) + k2_gain * heading_error));

		control.steer = steering_angle * 0.9549f; // times 180 / ( PI * 60) to return it to a float between -1 and 1, having 60 degrees my limit

        control.speed = 0.4f + 0.4f*(1-float(fabs(control.steer))); // min + (max-min) speed

		break;}

	case 1:{
		first_call = true;

		//trial1
		float cross_track_error_inpixels = (track_distance_unitless/2.0f) - float(fabs(window_center - vec1.m_x0));
		cross_track_error = 0.55f * cross_track_error_inpixels / track_distance_unitless;
		// cross_track_error will always be positive from this equation, so we check our position
		if(window_center > vec1.m_x0){ // car is left the desired path, vec1 is the left sided vector
			cross_track_error = (-1.f) * cross_track_error;
		}


		float slope = (float(vec1.m_x1-vec1.m_x0))/(float(vec1.m_y1-vec1.m_y0));
		heading_error = float(atan(slope));

		steering_angle = heading_error - float(atan( (float(k1_gain) * cross_track_error / (control.speed + 0.1f)) + k2_gain * heading_error));

		control.steer = steering_angle * 0.9549f;

        control.speed = 0.4f + 0.4f*(1-float(fabs(control.steer)));

		break;}
	}

	return control;
}