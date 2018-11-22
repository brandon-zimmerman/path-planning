#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const double WEIGHT_LANE_CHANGE = pow(10, 9);
const double WEIGHT_SPEED = pow(10, 5);


double goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
	/*
	Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
	Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
	*/
	double cost;
	double distance = data["distance_to_goal"];
	if (distance > 0) {
		cost = 1 - 2 * exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
	}
	else {
		cost = 1;
	}
	return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
	/*
	Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
	*/

	double proposed_speed_intended = lane_speed(vehicle, predictions, (int) data["intended_lane"]);
	if (proposed_speed_intended < 0 || proposed_speed_intended > vehicle.target_speed) {
		proposed_speed_intended = vehicle.target_speed;
	}

	double proposed_speed_final = lane_speed(vehicle, predictions, (int) data["final_lane"]);
	if (proposed_speed_final < 0 || proposed_speed_final > vehicle.target_speed) {
		proposed_speed_final = vehicle.target_speed;
	}

	double cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed;

	return cost;
}

double lane_speed(const Vehicle vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
	/*
	Calculate the lane speed based on the closets non-ego vehicle.	
	*/	
	double smallest_dist = 1000000;
	bool veh_found = false;
	Vehicle closest_veh;
	for (auto kp: predictions) {
		int key = kp.first;
		Vehicle other_vehicle = kp.second[0];			
		if (other_vehicle.lane == lane && key != -1) {
			double s_dist = other_vehicle.s - vehicle.s;
			if (abs(s_dist < 60) && s_dist > 0 && s_dist < smallest_dist) {
				closest_veh = other_vehicle;
				smallest_dist = s_dist;
				veh_found = true;
			}			
		}
	}
	if (veh_found) {
		return closest_veh.v;
	}
	//Found no vehicle in the lane
	return -1.0;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
	/*
	Sum weighted cost functions to get total cost for trajectory.
	*/
	map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
	double cost = 0.0;

	//Add additional cost functions here.
	vector< function<double(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = { inefficiency_cost, goal_distance_cost };
	vector<double> weight_list = { WEIGHT_LANE_CHANGE, WEIGHT_SPEED };

	for (int i = 0; i < cf_list.size(); i++) {
		double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
		cost += new_cost;
	}

	return cost;

}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
	/*
	Generate helper data to use in cost functions:
	indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
	final_lane: the lane of the vehicle at the end of the trajectory.
	distance_to_goal: the distance of the vehicle to the goal.

	Note that indended_lane and final_lane are both included to help differentiate between planning and executing
	a lane change in the cost functions.
	*/
	map<string, double> trajectory_data;
	Vehicle trajectory_last = trajectory[1];
	int intended_lane;

	if (trajectory_last.state == VehicleState::PrepareLaneChangeLeft) {
		intended_lane = trajectory_last.lane + 1;
	}
	else if (trajectory_last.state == VehicleState::PrepareLaneChangeRight) {
		intended_lane = trajectory_last.lane - 1;
	}
	else {
		intended_lane = trajectory_last.lane;
	}

	double distance_to_goal = vehicle.goal_s - trajectory_last.s;
	int final_lane = trajectory_last.lane;
	trajectory_data["intended_lane"] = intended_lane;
	trajectory_data["final_lane"] = final_lane;
	trajectory_data["distance_to_goal"] = distance_to_goal;
	return trajectory_data;
}

