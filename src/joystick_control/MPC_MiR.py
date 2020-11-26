#!/usr/bin/env python
# -*- coding: utf-8 -*-

from casadi import *
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import SetModelState
import time
# import thread


class MPC_Control(object):
    def __init__(self,N,dt):
        self._N = N
        self._dt = dt
        self._Q = SX(3,3)
        self._R = SX(2,2)
        self._r_diam = 0.95
        self._solver = None
        self._args = {}
        self._x0 = None
        self._xs = None
        self._X0 = None
        self._u0 = DM.zeros(N,2)
        self._G = []
        self._obs_diam = []
        self._obs_x = []
        self._obs_y = []
        self._X_R = 0.0
        self._Y_R = 0.0
        self._THETA_R = 0.0
        self._X_vR = 0.0
        self._Y_vR = 0.0
        self._THETA_vR = 0.0
        self._exit_flag = False

        self._pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self._pub_initial = rospy.Publisher('/initial_pose',Pose,queue_size=1)

        rospy.Subscriber("/end_sim", Bool, self.terminate_processes)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.update_mir_pose)
        rospy.Subscriber("/leader_point", PoseStamped, self.update_vmir_pose)


    def update_mir_pose(self, data):
        index = data.name.index('mir')
        quat_orientation = data.pose[index].orientation
        self._X_R = data.pose[index].position.x
        self._Y_R = data.pose[index].position.y
        quats = [quat_orientation.x, quat_orientation.y, quat_orientation.z,\
                 quat_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quats)
        self._THETA_R = yaw


    def update_vmir_pose(self, data):
        quat_orientation = data.pose.orientation
        self._X_vR = data.pose.position.x
        self._Y_vR = data.pose.position.y
        quats = [quat_orientation.x, quat_orientation.y, quat_orientation.z,\
                 quat_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quats)
        self._THETA_vR = yaw


    def terminate_processes(self, data):
        if data.data == 0:
            self._exit_flag = False
        else:
            self._exit_flag = True


    def get_obstacles_for_problem(self):
        """
        Define the obstacles for obstacle detection.
        Must be later substituted with dynamic obstacle detection.
        Return: 'List' of X and Y coordinate of obstacle with diameter of obstacle.
        """

        self._obs_x = [-4,-2,0.5,3,-2.5,0,2,-4,-1,3,-2,1,4,-1,1,3] # meters
        self._obs_y = [3.5,3,3.5,4,1.5,1.5,2,0,0,0.5,-2,-1,-1,-4,-3,-3] # meters
        self._obs_diam = [0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, \
                    0.45, 0.45, 0.45, 0.45, 0.45, 0.45]  # meters


    def initialize_nlp_solver(self):
        """
        Setup the NLP problem based on the formulated MPC problem using Casadi.
        Return: Solver object of class nlpsol.
        """
        # Weighting Matrices for the cost function
        self._Q[0,0] = 6
        self._Q[1,1] = 6
        self._Q[2,2] = 3

        self._R[0,0] = 0.3
        self._R[1,1] = 0.15

        # Define the state vector and control vectors
        x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta')
        state_vector = vertcat(x,y,theta); n_state = len(SX.elements(state_vector))

        v = SX.sym('v'); w = SX.sym('w')
        control_vector = vertcat(v,w); n_control = len(SX.elements(control_vector))

        # Define the ODE/DAE for the problem from robot kinematic model
        ode = vertcat(v*cos(theta), v*sin(theta), w)

        f= Function('f', [state_vector,control_vector], [ode])

        # Define the Optimization variables and paramters as CasADi symbolics.
        U = SX.sym('U',n_control,self._N)
        X = SX.sym('X',n_state,N+1)
        P = SX.sym('P',n_state+n_state)

        init_state = X[:,0]
        obj = 0
        diff = init_state-P[0:3]
        # compute symbolically the objective function and constraints
        for i in range(len(SX.elements(diff))):
            self._G.append(diff[i])

        for k in range(self._N):
            init_state = X[:,k]; control = U[:,k]
            obj = obj + (mtimes(mtimes(transpose(init_state-P[3:]), self._Q), \
                                (init_state-P[3:]))) + \
                mtimes(mtimes(transpose(control), self._R), control)
            next_state = X[:,k+1]; f_value = f(init_state,control)
            next_state_euler = init_state + (self._dt*f_value)
            diff = next_state-next_state_euler
            for i in range(len(SX.elements(diff))):
                self._G.append(diff[i])

        self.get_obstacles_for_problem()

        for k in range(self._N+1):
            for tot_obs in range(len(self._obs_x)):
                self._G.append(-sqrt((X[0,k]-self._obs_x[tot_obs])**2 + \
                               (X[1,k]-self._obs_y[tot_obs])**2)\
                         + ((self._r_diam/2) + (self._obs_diam[tot_obs]/2)))

        for k in range(self._N-1):
            self._G.append(U[0,k]-U[0,k+1])
        for k in range(self._N-1):
            self._G.append(U[1,k]-U[1,k+1])

        # ----------------- Define the optimization problem-------------------------

        OPT_Variables = vertcat(reshape(X,3*(self._N+1),1),reshape(U,(2*self._N),1))
        nlp = {"x":OPT_Variables,"f":obj, "g":vertcat(*self._G), "p":P}

        # Define the options for the solver
        opts = {"ipopt.max_iter":10000,\
                "ipopt.print_level":0, \
                "ipopt.acceptable_tol":1e-8, \
                "ipopt.acceptable_obj_change_tol":1e-7,\
                "print_time":0}
        # Define the NLP Solver

        self._solver = nlpsol("solver","ipopt", nlp,opts)


    def initialize_constraint_values(self):
        """
        Define the Values for the constraints of the formulated
        optimization problem.

        Return: Dictionary of constraint values.
        """

        # Define the limits for the linear and angular velocities
        v_max = 1; v_min=-v_max
        w_max = 1; w_min=-w_max

        # Define the limits for change in Linear and Angular Velocities
        vp_lims = 0.1; vn_lims = -0.1
        wp_lims = 0.1; wn_lims = -0.1

        # Define the constraints for the solver
        self._args["lbx"] = [None for i in range((N*3)+3+(2*self._N))]
        self._args["ubx"] = [None for i in range((N*3)+3+(2*self._N))]
        self._args["lbg"] = [None for i in range(len(self._G))]
        self._args["ubg"] = [None for i in range(len(self._G))]

        len_state_constraints = (self._N*3)+3
        len_control_constraints = len(self._args["lbx"])
        len_eq_const = 3*(self._N+1)
        len_obst_const = len_eq_const + len(self._obs_x)*(self._N+1)
        len_vd = len_obst_const+(self._N-1)
        len_wd = len_vd+(self._N-1)

        # Bounds on the state variables (Boundary Conditions/Outer walls)
        for i in range(0,len_state_constraints,3):
            self._args["lbx"][i] = -4.45
            self._args["ubx"][i] = 4.45

        for i in range(1,len_state_constraints,3):
            self._args["lbx"][i] = -4.45
            self._args["ubx"][i] = 4.45

        for i in range(2,len_state_constraints,3):
            self._args["lbx"][i] = -inf
            self._args["ubx"][i] = inf

        # Bounds on the control Variables (Linear and angular velocity)
        for i in range(len_state_constraints, len_control_constraints, 2):
            self._args["lbx"][i] = v_min
            self._args["ubx"][i] = v_max

        for i in range(len_state_constraints+1, len_control_constraints, 2):
            self._args["lbx"][i] = w_min
            self._args["ubx"][i] = w_max

        # equality constraints
        for i in range(0,len_eq_const):
            self._args["lbg"][i] = 0
            self._args["ubg"][i] = 0

        # obstacle constraints
        for i in range(len_eq_const,len_obst_const):
            self._args["lbg"][i] = -inf
            self._args["ubg"][i] = 0

        # linear velocity decomposition constraints
        for i in range(len_obst_const, len_vd):
            self._args["lbg"][i] = vn_lims
            self._args["ubg"][i] = vp_lims

        # angular velocity decomposition constraints
        for i in range(len_vd,len_wd):
            self._args["lbg"][i] = wn_lims
            self._args["ubg"][i] = wp_lims


    def initialize_gazebo_model_pose(self):

        init_R_x = -4.0
        init_R_y = -4.0
        init_R_th = 0.0

        self._x0 = vertcat(init_R_x, init_R_y, init_R_th)
        self._xs = self._x0

        quat = quaternion_from_euler(0,0,init_R_th)
        init_gazebo = ModelState()
        init_pose = Pose()
        init_gazebo.model_name = 'mir'
        init_gazebo.pose.position.x = init_R_x
        init_gazebo.pose.position.y = init_R_y
        init_gazebo.pose.orientation.x = quat[0]
        init_gazebo.pose.orientation.y = quat[1]
        init_gazebo.pose.orientation.z = quat[2]
        init_gazebo.pose.orientation.w = quat[3]

        init_pose.position.x = init_R_x
        init_pose.position.y = init_R_y
        init_pose.orientation.x = quat[0]
        init_pose.orientation.y = quat[1]
        init_pose.orientation.z = quat[2]
        init_pose.orientation.w = quat[3]

        self._pub_initial.publish(init_pose)
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state',
    					SetModelState)
            resp = set_state( init_gazebo )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        self._X0 = repmat(self._x0,1,self._N+1).T


    def MPC_control_loop(self):
        self._args["p"] = vertcat(self._x0,self._xs)
        self._args["x0"] = vertcat(reshape(self._X0.T,3*(self._N+1),1),\
                                    reshape(self._u0.T,2*self._N,1))

        # -------------------------- Run the solver --------------------------------

        sol = self._solver(lbx = vertcat(*self._args['lbx']),\
                     ubx = vertcat(*self._args['ubx']),\
                     lbg = horzcat(*self._args['lbg']),\
                     ubg = horzcat(*self._args['ubg']),\
                     p = self._args['p'],\
                     x0 = self._args['x0'])
        u = reshape(DM.full(sol['x'][3*(N+1):]).T,2,N).T

        twist = Twist()
        twist.linear.x = u[0,0]; twist.angular.z = u[0,1]
        self._pub_vel.publish(twist)

        # -----------------Propogate values for next step---------------------------

        self._x0 = vertcat(self._X_R, self._Y_R, self._THETA_R)
        self._xs = vertcat(self._X_vR, self._Y_vR, self._THETA_vR)
        self._u0 = vertcat(u[1:,:], u[-1,:])
        self._X0 = reshape(DM.full(sol["x"][0:(3*(self._N+1))]).T,3,self._N+1).T
        self._X0 = vertcat(self._X0[1:,:], self._X0[-1,:])


    def start(self):
        loop_dt = rospy.Rate(2)
        while self._exit_flag == False:
            self.MPC_control_loop()
            loop_dt.sleep()


if __name__ == '__main__':
    rospy.init_node('MPC_Python_Controller')
    N = 15; dt = 0.4
    mpc = MPC_Control(N, dt)
    mpc.initialize_nlp_solver()
    mpc.initialize_constraint_values()
    mpc.initialize_gazebo_model_pose()
    mpc.start()
