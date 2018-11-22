#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, int lane, double s, double v, double a, VehicleState state) {

	this->id = id;
	this->lane = lane;
	this->goal_lane = lane;
	this->s = s;
	this->v = v;
	this->a = a;
	this->state = state;	
	this->is_initialized = true;
	//cout << "id=" << this->id << "; s=" << this->s << "\n";

}

Vehicle::~Vehicle() {}

void Vehicle::update_state(int lane, double s, double v) {
	/*
	Called by simulator before simulation begins. Sets various
	parameters which will impact the ego vehicle.
	*/	
	this->lane = lane;	
	this->s = s;
	this->v = v;	
	//cout << "id=" << this->id << "; s=" << this->s << "\n";
}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
	/*
	INPUT: A predictions map. This is a map of vehicle id keys with predicted
		vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
		the vehicle at the current timestep and one timestep in the future.
	OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

	*/
	vector<VehicleState> states = successor_states();
	double cost;
	vector<double> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (auto state_val: states) {
		vector<Vehicle> trajectory = generate_trajectory(state_val, predictions);
		if (trajectory.size() != 0) {
			cost = calculate_cost(*this, predictions, trajectory);
			costs.push_back(cost);
			final_trajectories.push_back(trajectory);
		}
	}

	vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = (int) distance(begin(costs), best_cost);
	return final_trajectories[best_idx];
}

vector<VehicleState> Vehicle::successor_states() {
	/*
	Provides the possible next states given the current state for the FSM
	discussed in the course, with the exception that lane changes happen
	instantaneously, so LCL and LCR can only transition back to KL.
	*/
	vector<VehicleState> states;
	
	VehicleState state = this->state;	

	states.push_back(VehicleState::KeepLane);
	if ((state == VehicleState::LaneChangeLeft || state == VehicleState::LaneChangeRight) 
		&& (this->lane != this->goal_lane)) {
			states.push_back(state);			
	}	
	if (state == VehicleState::KeepLane) {
		if (lane != lanes_available - 1) {
			states.push_back(VehicleState::PrepareLaneChangeLeft);
		}
		if (lane != 0) {
			states.push_back(VehicleState::PrepareLaneChangeRight);
		}
	}
	else if (state == VehicleState::PrepareLaneChangeLeft) {
		if (lane != lanes_available - 1) {
			states.push_back(VehicleState::PrepareLaneChangeLeft);
			states.push_back(VehicleState::LaneChangeLeft);
		}
	}
	else if (state == VehicleState::PrepareLaneChangeRight) {
		if (lane != 0) {
			states.push_back(VehicleState::PrepareLaneChangeRight);
			states.push_back(VehicleState::LaneChangeRight);
		}		
	}

	
	//If state is "LCL" or "LCR", then just return "KL"
	return states;
}

vector<Vehicle> Vehicle::generate_trajectory(VehicleState state, map<int, vector<Vehicle>> predictions) {
	/*
	Given a possible next state, generate the appropriate trajectory to realize the next state.
	*/
	vector<Vehicle> trajectory;
	if (state == VehicleState::ConstantSpeed) {
		trajectory = constant_speed_trajectory();
	}
	else if (state == VehicleState::KeepLane) {
		trajectory = keep_lane_trajectory(predictions);
	}
	else if (state == VehicleState::LaneChangeLeft || state == VehicleState::LaneChangeRight) {
		trajectory = lane_change_trajectory(state, predictions);
	}
	else if (state == VehicleState::PrepareLaneChangeLeft || state == VehicleState::PrepareLaneChangeRight) {
		trajectory = prep_lane_change_trajectory(state, predictions);
	}
	return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
	/*
	Gets next timestep kinematics (position, velocity, acceleration)
	for a given lane. Tries to choose the maximum velocity and acceleration,
	given other vehicle positions and accel/velocity constraints.
	*/
	double max_velocity_accel_limit = (this->max_acceleration + this->v);
	double min_velocity_decel_limit = (this->v - max_acceleration);
	double new_position;
	double new_velocity;
	double new_accel;
	Vehicle vehicle_ahead;
	Vehicle vehicle_behind;

	if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

		//if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
		//	new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
		//}
		//else {
		double max_velocity_in_front = ((vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a));
		
		new_velocity = fmin(fmin(max_velocity_in_front, this->target_speed), max_velocity_accel_limit);

		if (new_velocity < this->v)
		{
			new_velocity = fmax(new_velocity, min_velocity_decel_limit);
		}
		//}
	}
	else {
		new_velocity = fmin(max_velocity_accel_limit, this->target_speed);
	}

	new_accel = (new_velocity - this->v) / this->t; //Equation: (v_1 - v_0)/t = acceleration
	new_position = this->s + new_velocity * this->t; // + new_accel / 2.0;
	return{ new_position, new_velocity, new_accel };

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
	/*
	Generate a constant speed trajectory.
	*/
	double next_pos = position_at(this->t);
	vector<Vehicle> trajectory = { Vehicle(this->id, this->lane, this->s, this->v, this->a, this->state),
								  Vehicle(this->id, this->lane, next_pos, this->v, 0, this->state) };
	return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
	/*
	Generate a keep lane trajectory.
	*/
	vector<Vehicle> trajectory = { Vehicle(this->id, lane, this->s, this->v, this->a, state) };
	vector<double> kinematics = get_kinematics(predictions, this->lane);
	double new_s = kinematics[0];
	double new_v = kinematics[1];
	double new_a = kinematics[2];
	trajectory.push_back(Vehicle(this->id, this->lane, new_s, new_v, new_a, VehicleState::KeepLane));
	return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(VehicleState state, map<int, vector<Vehicle>> predictions) {
	/*
	Generate a trajectory preparing for a lane change.
	*/
	double new_s;
	double new_v;
	double new_a;
	Vehicle vehicle_behind;
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory = { Vehicle(this->id, this->lane, this->s, this->v, this->a, this->state) };
	vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

	if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
		//Keep speed of current lane so as not to collide with car behind.
		new_s = curr_lane_new_kinematics[0];
		new_v = curr_lane_new_kinematics[1];
		new_a = curr_lane_new_kinematics[2];

	}
	else {
		vector<double> best_kinematics;
		vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
		//Choose kinematics with lowest velocity.
		if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
			best_kinematics = next_lane_new_kinematics;
		}
		else {
			best_kinematics = curr_lane_new_kinematics;
		}
		new_s = best_kinematics[0];
		new_v = best_kinematics[1];
		new_a = best_kinematics[2];
	}

	trajectory.push_back(Vehicle(this->id, this->lane, new_s, new_v, new_a, state));
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(VehicleState state, map<int, vector<Vehicle>> predictions) {
	/*
	Generate a lane change trajectory.
	*/

	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory;
	Vehicle next_lane_vehicle;
	//Check if a lane change is possible (check if another vehicle occupies that spot).
	for (auto kv: predictions)
	{
		next_lane_vehicle = kv.second[0];
		if (next_lane_vehicle.lane == new_lane && next_lane_vehicle.s + this->change_lanes_buffer > this->s && next_lane_vehicle.s - this->change_lanes_buffer < this->s) {
			//If lane change is not possible, return empty trajectory.
			return trajectory;
		}
	}
	trajectory.push_back(Vehicle(this->id, this->lane, this->s, this->v, this->a, this->state));
	vector<double> kinematics = get_kinematics(predictions, new_lane);
	trajectory.push_back(Vehicle(this->id, new_lane, kinematics[0], kinematics[1], kinematics[2], state));
	return trajectory;
}

double Vehicle::position_at(float t) {
	double s = this->s + this->v*t + this->a*t*t / 2.0;
	
	//cout << "id=" << this->id << "; v="<< this->v << "; a="<< a << "; t="<<t<< "; this->s="<< this->s << "; s=" << s << "\n";
	return s;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	/*
	Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
	rVehicle is updated if a vehicle is found.
	*/
	double max_s = -1;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (auto kv: predictions) {
		temp_vehicle = kv.second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
			max_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	/*
	Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
	rVehicle is updated if a vehicle is found.
	*/
	double min_s = this->goal_s;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (auto kv: predictions) {
		temp_vehicle = kv.second[0];
		//cout << "goal_s" << min_s << "; s=" << this->s << "; temp_vehicle_s=" << temp_vehicle.s <<"\n";
		if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s) {
			min_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
	/*
	Generates predictions for non-ego vehicles to be used
	in trajectory generation for the ego vehicle.
	*/
	vector<Vehicle> predictions;
	for (int i = 0; i < horizon; i++) {
		double next_s = position_at(i*this->t);
		double next_v = 0;
		if (i < horizon - 1) {
			next_v = position_at((i + 1) * this->t) - s;
		}
		predictions.push_back(Vehicle(this->id, this->lane, next_s, next_v, 0));
	}
	return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
	/*
	Sets state and kinematics for ego vehicle using the last state of the trajectory.
	*/
	Vehicle next_state = trajectory[1];
	this->state = next_state.state;
	this->lane = next_state.lane;
	this->s = next_state.s;
	this->v = next_state.v;
	this->a = next_state.a;
}

void Vehicle::configure(vector<int> road_data) {
	/*
	Called by simulator before simulation begins. Sets various
	parameters which will impact the ego vehicle.
	*/
	this->target_speed = road_data[0];
	this->lanes_available = road_data[1];	
	this->max_acceleration = road_data[2];
}


bool Vehicle::initialized()
{
	return is_initialized;
}



std::ostream &operator << (std::ostream &os, const VehicleState &state)
{
	switch (state)
	{
	case KeepLane: os << "KeepLane"; break;
	case ConstantSpeed: os << "ConstantSpeed"; break;
	case PrepareLaneChangeLeft: os << "PrepareLaneChangeLeft"; break;
	case PrepareLaneChangeRight: os << "PrepareLaneChangeRight"; break;
	case LaneChangeLeft: os << "LaneChangeLeft"; break;
	case LaneChangeRight: os << "LaneChangeRight"; break;
	}
	return os;
}