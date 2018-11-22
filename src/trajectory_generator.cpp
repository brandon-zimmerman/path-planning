#include <vector>
#include "trajectory_generator.h"
#include "behavior_planner.h"
#include "spline.h"
#include "helper.h"


vector<vector<double>> TrajectoryGenerator::generate_next_trajectory(double car_x, double car_y,
	double car_s, double car_yaw, vector<double>& previous_path_x,
	vector<double>& previous_path_y, vector<double>& map_waypoints_s,
	vector<double>& map_waypoints_x, vector<double>& map_waypoints_y, 
	int target_lane, float target_velocity)
{
	int prev_size = (int) previous_path_x.size();

	// Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
	// later we will interpolate these waypoints with a spline and fill it in with more points that control 

	vector<double> ptsx;
	vector<double> ptsy;

	// reference x, y, yaw stats
	// either we will reference trhe starting point as where the car is or at the previous paths end point
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = Helper::deg2rad(car_yaw);

	if (prev_size < 2)
	{
		// use two points that make the path tangent to the car
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	}
	// use the previous path's end point as the starting reference
	else
	{
		// redefine reference state as previous path end point
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		// use two points that make the path tangent to the previous path's end point
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	double target_d = BehaviorPlanner::lane_to_d(target_lane);

	vector<double> next_wp0 = Helper::getXY(car_s + 50, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = Helper::getXY(car_s + 80, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = Helper::getXY(car_s + 110, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (unsigned int i = 0; i < ptsx.size(); i++)
	{
		// shift car reference angle top 0 degrees
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}

	// create a spline.
	tk::spline s;

	// set (x,y) points to the spline.
	s.set_points(ptsx, ptsy);

	// define the actual (x,y) points we will use for the planner
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	// start with all of the previous path points from last time
	for (unsigned int i = 0; i < previous_path_x.size(); i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// calculate how to break up spline points so that we travel at our desired reference velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

	double x_add_on = 0;

	// fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
	for (unsigned int i = 0; i < 50 - previous_path_x.size(); i++)
	{

		double N = (target_dist / (.02*target_velocity / 2.24));
		double x_point = x_add_on + (target_x) / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// roate back to normal after rotating it earlier.
		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);		
	}

	vector<vector<double>> next_vals;
	next_vals.push_back(next_x_vals);
	next_vals.push_back(next_y_vals);

	return next_vals;
}