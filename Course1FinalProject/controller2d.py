#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
from collections import deque 
import math
from casadi import *

class Controller2D(object):
	def __init__(self, waypoints):
		self.vars                = cutils.CUtils()
		self._current_x          = 0
		self._current_y          = 0
		self._current_yaw        = 0
		self._current_speed      = 0
		self._desired_speed      = 0
		self._current_frame      = 0
		self._current_timestamp  = 0
		self._start_control_loop = False
		self._set_throttle       = 0
		self._set_brake          = 0
		self._set_steer          = 0
		self._waypoints          = waypoints
		self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
		self._pi                 = np.pi
		self._2pi                = 2.0 * np.pi
		self._e_buffer = deque(maxlen=30)

	def update_values(self, x, y, yaw, speed, timestamp, frame):
		self._current_x         = x
		self._current_y         = y
		self._current_yaw       = yaw
		self._current_speed     = speed
		self._current_timestamp = timestamp
		self._current_frame     = frame
		if self._current_frame:
			self._start_control_loop = True

	def update_desired_speed(self):
		min_idx       = 0
		min_dist      = float("inf")
		desired_speed = 0
		for i in range(len(self._waypoints)):
			dist = np.linalg.norm(np.array([
					self._waypoints[i][0] - self._current_x,
					self._waypoints[i][1] - self._current_y]))
			if dist < min_dist:
				min_dist = dist
				min_idx = i
		if min_idx < len(self._waypoints)-1:
			desired_speed = self._waypoints[min_idx][2]
		else:
			desired_speed = self._waypoints[-1][2]
		self._desired_speed = desired_speed

	def update_waypoints(self, new_waypoints):
		self._waypoints = new_waypoints

	def get_commands(self):
		return self._set_throttle, self._set_steer, self._set_brake

	def set_throttle(self, input_throttle):
		# Clamp the throttle command to valid bounds
		throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
		self._set_throttle = throttle

	def set_steer(self, input_steer_in_rad):
		# Covnert radians to [-1, 1]
		input_steer = self._conv_rad_to_steer * input_steer_in_rad

		# Clamp the steering command to valid bounds
		steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
		self._set_steer = steer

	def set_brake(self, input_brake):
		# Clamp the steering command to valid bounds
		brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
		self._set_brake = brake

	def pid(self):
		print("=============== PID Controller ===============")

		######################################################
		# RETRIEVE SIMULATOR FEEDBACK
		######################################################
		x               = self._current_x
		y               = self._current_y
		yaw             = self._current_yaw
		v               = self._current_speed
		self.update_desired_speed()
		v_desired       = self._desired_speed
		t               = self._current_timestamp
		waypoints       = self._waypoints
		throttle_output = 0
		steer_output    = 0
		brake_output    = 0

		######################################################
		######################################################
		# MODULE 7: DECLARE USAGE VARIABLES HERE
		######################################################
		######################################################
		"""
			Use 'self.vars.create_var(<variable name>, <default value>)'
			to create a persistent variable (not destroyed at each iteration).
			This means that the value can be stored for use in the next
			iteration of the control loop.

			Example: Creation of 'v_previous', default value to be 0
			self.vars.create_var('v_previous', 0.0)

			Example: Setting 'v_previous' to be 1.0
			self.vars.v_previous = 1.0

			Example: Accessing the value from 'v_previous' to be used
			throttle_output = 0.5 * self.vars.v_previous
		"""
		self.vars.create_var('v_previous', 0.0)

		# Skip the first frame to store previous values properly
		if self._start_control_loop:
			"""
				Controller iteration code block.

				Controller Feedback Variables:
					x               : Current X position (meters)
					y               : Current Y position (meters)
					yaw             : Current yaw pose (radians)
					v               : Current forward speed (meters per second)
					t               : Current time (seconds)
					v_desired       : Current desired speed (meters per second)
									  (Computed as the speed to track at the
									  closest waypoint to the vehicle.)
					waypoints       : Current waypoints to track
									  (Includes speed to track at each x,y
									  location.)
									  Format: [[x0, y0, v0],
											   [x1, y1, v1],
											   ...
											   [xn, yn, vn]]
									  Example:
										  waypoints[2][1]: 
										  Returns the 3rd waypoint's y position

										  waypoints[5]:
										  Returns [x5, y5, v5] (6th waypoint)
				
				Controller Output Variables:
					throttle_output : Throttle output (0 to 1)
					steer_output    : Steer output (-1.22 rad to 1.22 rad)
					brake_output    : Brake output (0 to 1)
			"""

			######################################################
			######################################################
			# MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
			######################################################
			######################################################
			"""
				Implement a longitudinal controller here. Remember that you can
				access the persistent variables declared above here. For
				example, can treat self.vars.v_previous like a "global variable".
			"""
			
			# Change these outputs with the longitudinal controller. Note that
			# brake_output is optional and is not required to pass the
			# assignment, as the car will naturally slow down over time.
			
			# Longitudinal Controller parameters 
			K_P = 1.5
			K_D = 0.01
			K_I = 1.0
			dt = 0.05
			
			# speed error
			_e = (v_desired - v)
			self._e_buffer.append(_e)

			# print("v: ", v)
			# print("v_desired: ", v_desired)

			if len(self._e_buffer) >= 2:
				_de = (self._e_buffer[-1] - self._e_buffer[-2]) / dt
				_ie = sum(self._e_buffer) * dt
			else:
				_de = 0.0
				_ie = 0.0
		
			# control signal
			u =  np.clip((K_P * _e) + (K_D * _de / dt) + (K_I * _ie * dt), -1.0, 1.0)

			if (u<=0):
				throttle_output = 0
				brake_output    = -u
			else:
				throttle_output = u
				brake_output    = 0

			######################################################
			######################################################
			# MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
			######################################################
			######################################################
			"""
				Implement a lateral controller here. Remember that you can
				access the persistent variables declared above here. For
				example, can treat self.vars.v_previous like a "global variable".
			"""
			
			# Parameters for Lateral controller
			k = 0.1         #look forward gain
			Lfc = 1.0       #look-ahead distance
			L = 2.9

			# search nearest point index   
			length = np.arange(0,100,1)

			dx = [x - waypoints[icx][0] for icx in length]
			dy = [y - waypoints[icy][1] for icy in length]
			d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx,idy) in zip(dx,dy)]
			ind = d.index(min(d))


			if ind < 2:
				tx = waypoints[ind][0] # target point
				ty = waypoints[ind][1]  
			else:
				tx = waypoints[-1][0]
				ty = waypoints[-1][1]

			alpha_hat = math.atan2(ty - y,tx - x) # angle between target and current point
			alpha = alpha_hat - yaw

			Lf = k * v + Lfc

			# Change the steer output with the lateral controller. 
			steer_output = math.atan2(2.0 * L * math.sin(alpha) / Lf,1.0)

			######################################################
			# SET CONTROLS OUTPUT
			######################################################
			print("--------------------------------------")
			print("steer_output: ", steer_output)
			print("throttle_output: ", throttle_output)
			print("brake_output: ", brake_output)
			print("--------------------------------------")
			print("   ")

			self.set_throttle(throttle_output)  # in percent (0 to 1)
			self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
			self.set_brake(brake_output)        # in percent (0 to 1)

		######################################################
		######################################################
		# MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
		######################################################
		######################################################
		"""
			Use this block to store old values (for example, we can store the
			current x, y, and yaw values here using persistent variables for use
			in the next iteration)
		"""
		self.vars.v_previous = v  # Store forward speed to be used in next step

	def get_cross_track_error(self, current_xy, waypoints):
	
		squared_terms = (current_xy - waypoints[:, :2])**2
		crosstrack_error = np.min(squared_terms[:, 0] + squared_terms[:, 1])

		yaw_cross_track = np.arctan2(current_xy[1] - waypoints[0][1], current_xy[0] - waypoints[0][0])
		yaw_path = np.arctan2(waypoints[-1][1] - waypoints[0][1], waypoints[-1][0] - waypoints[0][0])
		
		yaw_path2ct = yaw_path - yaw_cross_track
			   
		if yaw_path2ct > np.pi:
			yaw_path2ct -= 2 * np.pi
		if yaw_path2ct < - np.pi:
			yaw_path2ct += 2 * np.pi

		if yaw_path2ct > 0:
			crosstrack_error = abs(crosstrack_error)
		else:
			crosstrack_error = - abs(crosstrack_error)

		return crosstrack_error    

	def get_psi(self, vehicle_transform):

		v_begin = vehicle_transform.location

		v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
										 y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

		# vehicle heading vector
		v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
		
		yaw_vehicle = np.arctan2(v_vec[1], v_vec[0])

		return yaw_vehicle

	def get_epsi(self, yaw, waypoints):
		yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
		yaw_diff = yaw_path - yaw 
		if yaw_diff > np.pi:
			yaw_diff -= 2 * np.pi
		if yaw_diff < - np.pi:
			yaw_diff += 2 * np.pi

		return yaw_diff

	def get_coeffs(self, waypoints_xy):

		x = np.array(waypoints_xy)[:,0]
		y = np.array(waypoints_xy)[:,1]

		coeffs = np.flip(np.polyfit(x, y, 3))

		return coeffs

	def mpc(self):
		print("=============== MPC Controller ===============")

		######################################################
		# RETRIEVE SIMULATOR FEEDBACK
		######################################################
		x               = self._current_x
		y               = self._current_y
		yaw             = self._current_yaw
		v               = self._current_speed
		self.update_desired_speed()
		v_desired       = self._desired_speed
		t               = self._current_timestamp
		waypoints       = self._waypoints
		throttle_output = 0
		steer_output    = 0
		brake_output    = 0

		# Transform points from world frame to car frame
		_x = x # vehicle_transform.location.x
		_y = y # vehicle_transform.location.y
		
		# _psi = vehicle_transform.rotation.yaw
		_psi = yaw #self.get_psi(vehicle_transform)

		R = np.array([[math.cos(-_psi), -math.sin(-_psi)],[math.sin(-_psi), math.cos(-_psi)]])
		t = np.array([[_x],[_y]])

		waypoints_world = np.array(waypoints)
		waypoints_car = np.array(waypoints)

		# print("waypoints_world: ", waypoints_world)
		# print("-----------------------------------------")

		waypoints_car[:, 0:2] = np.dot(R, np.array([waypoints_world[:, 0], waypoints_world[:, 1]]) - t).T
		# print("waypoints_car: ", waypoints_car)

		N = 10
		dt = 0.1
		Lf = 2.67
		ref_v = v_desired

		# Define var start positions 
		x_start = 0
		y_start = x_start + N
		psi_start = y_start + N
		v_start = psi_start + N
		cte_start = v_start + N
		epsi_start = cte_start + N
		delta_start = epsi_start + N
		a_start = delta_start + N - 1

		# State
		x = 0
		y = 0
		psi = 0
		# v = v
		cte = self.get_cross_track_error([x,y], waypoints_car)

		epsi = self.get_epsi(psi, waypoints_car)
		coeffs = self.get_coeffs(waypoints_car)

		# number of model variables 
		# For example: If the [state] is a 4 element vector, the [actuators] is a 2   
		# element vector and there are 10 timesteps. The number of variables is:
		n_vars = N*6 + (N-1)*2
		
		# Set the number of constraints
		n_constraints = N*6
		
		# NLP variable vector
		vars =  MX.sym('x',n_vars)
		vars_init = np.zeros(n_vars)
		
		# set initial variables values
		vars_init[x_start] = x
		vars_init[y_start] = y
		vars_init[psi_start] = psi
		vars_init[v_start] = v
		vars_init[cte_start] = cte
		vars_init[epsi_start] = epsi
		
		# upperbound and lowerbound vectors for vars
		vars_upperbound = np.zeros(n_vars)
		vars_lowerbound = np.zeros(n_vars)
		
		# Set all non-actuators upper and lowerlimits
		# to the max negative and positive values.
		vars_upperbound[:delta_start] =  1.0e9
		vars_lowerbound[:delta_start] = -1.0e9
		
		# Set the upper and lower limits of delta as -25 and 25 degrees
		vars_upperbound[delta_start:a_start] =  0.7
		vars_lowerbound[delta_start:a_start] = -0.7
		
		# Set the upper and lower limits of accelerations as -1 and 1
		vars_upperbound[a_start:] =  1
		vars_lowerbound[a_start:] = -1

		# Lower and upper limits for the constraints 
		# Should be 0 besides initial state.
		constraints_upperbound = np.zeros(n_constraints)
		constraints_lowerbound = np.zeros(n_constraints)
		
		constraints_lowerbound[x_start] = x
		constraints_lowerbound[y_start] = y
		constraints_lowerbound[psi_start] = psi
		constraints_lowerbound[v_start] = v
		constraints_lowerbound[cte_start] = cte
		constraints_lowerbound[epsi_start] = epsi
		
		constraints_upperbound[x_start] = x
		constraints_upperbound[y_start] = y
		constraints_upperbound[psi_start] = psi
		constraints_upperbound[v_start] = v
		constraints_upperbound[cte_start] = cte
		constraints_upperbound[epsi_start] = epsi
		
		# Object for defining objective and constraints
		f, g = self.operator(vars, coeffs, n_constraints, N, dt, ref_v, Lf)

		# NLP
		nlp = {'x':vars, 'f':f, 'g':vertcat(*g)}
		# print("g shape:", vertcat(*g).shape) 
		
		## ----
		## SOLVE THE NLP
		## ----

		# Set options
		opts = {}
		opts["expand"] = True
		#opts["ipopt.max_iter"] = 4
		opts["ipopt.linear_solver"] = 'ma27'
		opts["ipopt.print_level"] = 0

		# Allocate an NLP solver
		solver = nlpsol("solver", "ipopt", nlp, opts)
		arg = {}

		# Initial condition
		arg["x0"] = vars_init
		# print("x0 shape: ", vars_init.shape)
		
		# Bounds on x
		arg["lbx"] = vars_lowerbound
		arg["ubx"] = vars_upperbound
		# print("lbx shape: ", vars_lowerbound.shape)
		
		# Bounds on g
		arg["lbg"] = constraints_lowerbound
		arg["ubg"] = constraints_upperbound
		# print("ubg: ", constraints_upperbound.shape)

		# Solve the problem
		res = solver(**arg)
		vars_opt = np.array(res["x"])

		x_mpc = vars_opt[x_start:y_start]
		y_mpc = vars_opt[y_start:psi_start]

		steering = vars_opt[delta_start:a_start]
		accelerations = vars_opt[a_start:]

		# print("steering: ", steering)
		# print("accelerations: ", accelerations)
		
		throttle_output = accelerations*0
		brake_output = accelerations*0

		for i in range(N-1):
			if accelerations[i]>0:
				throttle_output[i] = accelerations[i] 
				brake_output[i] = 0
			
			else:   
				throttle_output[i] = 0
				brake_output[i] = -accelerations[i]

		steer_output = steering
		
		print("--------------------------------------")
		print("Cross Track Error: ", cte)
		print("Heading Error: ", epsi)

		print("steer_output: ", steer_output[0][0])
		print("throttle_output: ", throttle_output[0][0])
		print("brake_output: ", brake_output[0][0])
		print("--------------------------------------")
		print("   ")

		# return throttle_output[0], brake_output[0], steer_output[0]
		self.set_throttle(throttle_output[0][0])  # in percent (0 to 1)
		self.set_steer(steer_output[0][0])        # in rad (-1.22 to 1.22)
		self.set_brake(brake_output[0][0])        # in percent (0 to 1)

	def operator(self, vars, coeffs, n_constraints, N, dt, ref_v, Lf):
		# Define var start positions 
		x_start = 0
		y_start = x_start + N
		psi_start = y_start + N
		v_start = psi_start + N
		cte_start = v_start + N
		epsi_start = cte_start + N
		delta_start = epsi_start + N
		a_start = delta_start + N - 1


		# fg = np.zeros(self.n_vars)
		f = MX.zeros(1)
		g = [0]*n_constraints
		
		# Add Cross track error, Heading error and velocity error to Cost Function
		for i in range(N):
			f[0] += 10000*vars[cte_start + i]**2
			f[0] += 10000*(vars[epsi_start + i])**2        ## 15
			f[0] += 1000*(vars[v_start + i] - ref_v)**2  ## 10,000    
					
		# Add control signal regulaization term to the Cost function
		for i in range(N-1):
			f[0] += 50*vars[delta_start + i]**2     # 100
			f[0] += 50*vars[a_start + i]**2     # 100
			# f[0] += 700*(vars[delta_start + i] * vars[v_start+i])**2
		
		# # Add cost for drastically changing controls 
		for i in range(N-2):
			f[0] += 250000*(vars[delta_start + i + 1] - vars[delta_start + i])**2
			f[0] += 200000*(vars[a_start + i + 1] - vars[a_start + i])**2
		
		# Add contraints
		g[x_start] = vars[x_start]
		g[y_start] = vars[y_start]
		g[psi_start] = vars[psi_start]
		g[v_start] = vars[v_start]
		g[cte_start] = vars[cte_start]
		g[epsi_start] = vars[epsi_start]

		for i in range(1,N):
			x1 = vars[x_start + i]
			y1 = vars[y_start + i]
			psi1 = vars[psi_start + i]
			v1 = vars[v_start + i]
			cte1 = vars[cte_start + i]
			epsi1 = vars[epsi_start + i]

			x0 = vars[x_start + i - 1]
			y0 = vars[y_start + i - 1]
			psi0 = vars[psi_start + i - 1]
			v0 = vars[v_start + i - 1]
			cte0 = vars[cte_start + i - 1]
			epsi0 = vars[epsi_start + i - 1]
			
			delta0 = vars[delta_start + i - 1]
			a0 = vars[a_start + i - 1]
					
			f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0**2 + coeffs[3] * x0 **3
			psides0 = atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0**2)                

			g[x_start + i] = x1 - (x0 + v0 * cos(psi0) * dt)
			g[y_start + i] = y1 - (y0 + v0 * sin(psi0) * dt)
			g[psi_start + i] = psi1 - (psi0 + (v0/Lf) * delta0 * dt)
			g[v_start + i] = v1 - (v0 + a0 * dt)
			g[cte_start + i] = cte1 - ((f0 - y0) + (v0 * sin(epsi0) * dt))
			g[epsi_start + i] = epsi1 - ((psi0 - psides0) + (v0/Lf) * delta0 * dt)
			
		return f, g   