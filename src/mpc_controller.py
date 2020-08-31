#!/usr/bin/env python2

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from kobuki_msgs.msg import MotorPower
from MPC import MPC

N = 10
N_c = 10
Ts = 0.1
X = np.array([0., 0.])
orientation = 0
V = np.array([0., 0.])
V_min = -1
V_max = 1
nx = 4

goal = np.zeros(2)
robot = sys.argv[1]
path = Path()
setpoint = np.zeros((N+1)*nx)

def accelerationTransform(a, v, w, theta_0):
    """This function applies the linearization transformation on acceleration values 
    based on equation 2.11
    """
    d = 0.2
    cos_theta = np.cos(theta_0)
    sin_theta = np.sin(theta_0)
    inverse = np.linalg.inv(np.array([[cos_theta, -d * sin_theta],[sin_theta, d * cos_theta]]))
    term1 = a[0] + v * w * sin_theta + d * (w**2) * cos_theta
    term2 = a[1] - v * w * cos_theta + d * (w**2) * sin_theta
    acc = np.matmul(inverse, np.vstack([term1, term2]))
    acc = acc.T

    return acc[0]

def updateWorld(msg):
    """This funcion is called whenever the gazebo/model_states publishes. This function
    updates the world variables as fast as possible"""
    global X, V, orientation
    d = 0.2
    
    X = np.array([float(msg.pose[-1].position.x), float(msg.pose[-1].position.y)])
    V = np.array([float(msg.twist[-1].linear.x), float(msg.twist[-1].linear.y)])
    orientation = np.arctan2(2 * float(msg.pose[-1].orientation.w) * float(msg.pose[-1].orientation.z), \
        1 - 2 * float(msg.pose[-1].orientation.z)**2)
    X[0] = X[0] + d*np.cos(orientation)
    X[1] = X[1] + d*np.sin(orientation)
    V[0] = V[0] + d*(-float(msg.twist[-1].angular.z))*np.sin(orientation)
    V[1] = V[1] + d*(float(msg.twist[-1].angular.z))*np.cos(orientation)

def cb_goal(msg):
    global V_des
    
    #path = Path()

    goal[0] = msg.goal.target_pose.pose.position.x
    goal[1] = msg.goal.target_pose.pose.position.y

	# Trajectory planning
    initial = np.copy(X)
    t0 = 5.0
    growth = 1
    logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
    d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
    V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)
    t = 0
    
    pub_motor_power.publish(0)

    data = rospy.wait_for_message('/move_base/NavfnROS/plan', Path)
    cb_path(data)
    
def cb_path(msg):
    if len(msg.poses) >= 1:
        global path
        sub_sampling = 5
        
        path = Path()
        
        for k in range(0,len(msg.poses),sub_sampling):
            path.poses.append(msg.poses[k])
        path.poses[-1] = msg.poses[-1]
    else:
        print("OLOCO")
 
rospy.init_node('mpc_controller')

# Velocity publishers
pub_motor_power = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=10)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

# Waiting gazebo first goal message
data = rospy.wait_for_message('/move_base/goal', MoveBaseActionGoal)
cb_goal(data)

# Subscribing on model_states instead of robot/odom, to avoid unnecessary noise
rospy.Subscriber('/gazebo/model_states', ModelStates, updateWorld)

# Subscribing to full path
#rospy.Subscriber('/move_base/NavfnROS/plan', Path, cb_path)

# Subscribing to goal
rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, cb_goal)

# Setpoint Publishers
pub_setpoint_pos = rospy.Publisher('/setpoint_pos', Vector3, queue_size=10)
pub_setpoint_vel = rospy.Publisher('/setpoint_vel', Vector3, queue_size=10)

setpoint_pos = Vector3()
setpoint_vel = Vector3()

# Initializing Controllers
controller = MPC(X, V_min, V_max, N, N_c, Ts)

# Trajectory planning
initial = np.copy(X)
t0 = 5.0
growth = 1
logistic = lambda t: 1/(1 + np.exp(- growth * (t - t0)))
d_logistic = lambda t: growth * logistic(t) * (1 - logistic(t))
V_des = lambda t: goal * d_logistic(t) - initial * d_logistic(t)
t = 0

vel = Twist()

while not rospy.is_shutdown():

    # Updating setpoint trajectory
    setpoint = np.zeros((N+1,nx))
    for k in range(0, N+1):
        if k >= len(path.poses):
            setpoint[k][:] = np.array([setpoint[k-1][0], setpoint[k-1][1], V_des(t + k * Ts)[0], V_des(t + k * Ts)[1]])
        else:
            setpoint[k][:] = np.array([path.poses[k].pose.position.x, path.poses[k].pose.position.y, V_des(t + k * Ts)[0], V_des(t + k * Ts)[1]])
    setpoint = np.ravel(setpoint)
    print(setpoint)
    
    if len(path.poses) > 1:
        path.poses.pop(0)

    # Updating initial conditions
    controller.x_0 = np.array([X[0], X[1], V[0], V[1]])

    # Computing optimal input values
    [velocity, acceleration] = controller.getNewVelocity(setpoint)

    if len(setpoint) > 1:
        [setpoint_pos.x, setpoint_pos.y] = setpoint[0:2]#P_des(t)
    else:
        [setpoint_pos.x, setpoint_pos.y] = [goal[0], goal[1]]

    [setpoint_vel.x, setpoint_vel.y] = V_des(t)
    
    print (">>",velocity[0], velocity[1])

    acc = accelerationTransform(acceleration, vel.linear.x, vel.angular.z, orientation)

    vel.linear.x = vel.linear.x + acc[0] * Ts
    vel.angular.z = vel.angular.z + acc[1] * Ts
        
    print ("$$", vel.linear.x, vel.angular.z)

    pub_motor_power.publish(1)
    pub.publish(vel)

    pub_setpoint_pos.publish(setpoint_pos)
    pub_setpoint_vel.publish(setpoint_vel)
    rospy.sleep(Ts)

    t += Ts
