#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H
#include <vector>

using namespace std;

class TrajectoryGenerator {
public:

	static vector<vector<double>> generate_next_trajectory(double car_x, double car_y,
		double car_s, double car_yaw, vector<double>& previous_path_x,
		vector<double>& previous_path_y, vector<double>& map_waypoints_s,
		vector<double>& map_waypoints_x, vector<double>& map_waypoints_y,
		int target_lane, float target_velocity);
	
};

#endif