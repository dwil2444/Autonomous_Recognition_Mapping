#!/usr/bin/env/python
import rospy
import numpy as np
import math
import cv2
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scipy import ndimage
from tf.transformations import euler_from_quaternion
# from apriltag_ros.msg import AprilTagDetectionArray

### Control Tools
from Controllers import PIDController
### State Machine Tools
from StateMachine import StateMachine


class Vault:
    """Class for storing variables that need to be shared across Callbacks. To be declared as global at start of script"""
    
    def __init__(self,**kwargs):
        self.params=kwargs
        
    def addParameters(self,**kwargs):
        self.params.update(kwargs)

    def get(param):
        return self.params[param]



# def detectRightCorner(lidarData):
#     i = 719
#     while lidarData.ranges[i-1] != float('inf') and i > 0:
#         i -= 1
#     phi = lidarData.angle_increment * i
#     x = lidarData.ranges[i] * math.cos(phi)
#     y = lidarData.ranges[i] * math.sin(phi)
#     return x,y

def my_center_of_mass(matrix):
    c = 0
    x = 0
    y = 0
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            y += i* matrix[i][j]
            x += j* matrix[i][j]
            c += matrix[i][j]
    return(y/c, x/c)

def detectCorner(lidarData,sweep,cw=True):
    end=sweep[1]
    i=sweep[0]
    if cw==True: #Cw is like detectLeftCorner
        while lidarData.ranges[i+1] != float('inf') and i <end:
            i+=1
    else:
        while lidarData.ranges[i-1] != float('inf') and i >end:
            i-=1
    if i==end:
        return 0,0
    phi = lidarData.angle_increment * i
    x = lidarData.ranges[i] * math.cos(phi)
    y = lidarData.ranges[i] * math.sin(phi)
    return x,y

def detectBoundary(lidarData,turn_direction):
    if turn_direction == -1:
        return lidarData.ranges[180]
    else:
        return lidarData.ranges[540]

# def detectLeftCorner(lidarData):
#     i = 0
#     while lidarData.ranges[i+1] != float('inf') and i < 719:
#         i += 1
#     phi = lidarData.angle_increment * i
#     x = lidarData.ranges[i] * math.cos(phi)
#     y = lidarData.ranges[i] * math.sin(phi)
#     return x,y

# def BdetectNearestCorner(lidarData):
#     xl,yl=detectLeftCorner(lidarData)
#     xr,yr=detectRightCorner(lidarData)
#     print("xl: %0.2f | yl: %0.2f | xr: %0.2f | yr: %0.2f"%(xl,yl,xr,yr))
#     dl=np.linalg.norm([xl,yl])
#     dr=np.linalg.norm([xr,yr])
#     if dl>dr:
#         return xr,yr,-1
#     else:
#         return xl,yl,1

def detectNearestCorner(lidarData):
    xl,yl=detectCorner(lidarData,[0,719],cw=True)
    xr,yr=detectCorner(lidarData,[719,0],cw=False)
    # print("xl: %0.2f | yl: %0.2f | xr: %0.2f | yr: %0.2f"%(xl,yl,xr,yr))
    dl=np.linalg.norm([xl,yl])
    dr=np.linalg.norm([xr,yr])
    if dl>dr:
        return xr,yr,-1
    else:
        return xl,yl,1

# def BcreateWayPoint(xc,yc,turn):
#     op_x = SM.getParam('x') + xc
#     op_y = SM.getParam('y') + yc

#     new_x = op_x - 0.2
#     new_y = op_y + 0.2*turn
#     return new_x, new_y

def createWayPoint(xc,yc,turn,bias_x,bias_y):
    op_x = SM.getParam('x') + xc
    op_y = SM.getParam('y') + yc

    new_x = op_x + bias_x
    new_y = op_y + bias_y*turn
    return new_x, new_y


### Callbacks
def cameraCallback(data):
    global SM

    """Callback for managing our camera"""
    #Common Behavior
    height = data.layout.dim[0].size
    width = data.layout.dim[1].size
    matrix = np.array(data.data).reshape(height, width)

    CoM=ndimage.measurements.center_of_mass(matrix/255)
    # CoM = my_center_of_mass(matrix/255)
    line_in_frame=(not np.isnan(CoM[0])) and (not np.isnan(CoM[1])) #GB

    #state dependent
    state=SM.getState()
    if state == "LineFollow":
        #Currently using CoM method; could look at hough space if issues
        if line_in_frame == True:
            SM.setParam('horz_center',CoM[1]) #GB
            # vert_center=CoM[0] #Currently unneeded for control
        else:
            pass    

    elif state =="RegainLine":
        #can just use line_in_frame variable again
        pass
    else : #i.e. halt
        pass


def lidarCallback(data):
    global SM
    # line_follow_speed = 0.6 # MANUALLY NEED TO CHANGE V_REF IN MAIN INSIDE THE STATE MACHINE
    line_follow_speed = SM.getParam('vel_ref')
    state=SM.getState()
    if state == "LineFollow": # and SM.getParam('obstacle_detected') == False:
        if data.ranges[0] < line_follow_speed: # PROPORTIONAL TO SPEED!!
            SM.setParam('obstacle_detected', True) #GB
            SM.setState('PivotInPlace')
            SM.setParam('turn_count',1)
            SM.setParam('vel_ref', 0.4)
        pass

    elif state =="DriveUntilClear":
        # print('lidaring')
        range1=560
        range2=600
        ### NEW CODE - Check all values between ranges (more robust than checking 2 values only) ### 
        cr = True
        for r in range(range1,range2,1):
            if data.ranges[r] < 1:
                cr = False
        ### END NEW CODE ##
        # cr = (data.ranges[range1]>1 and data.ranges[range2] > 1)
        if SM.getParam('turn_count') == 1 and cr == False: #After initial Turn
            print("looking for 1st Corner")
            SM.setParam('vel_ref', 0.2)
            # corner_reached = (data.ranges[range1]>1 and data.ranges[range2] > 1)
            # print("CORNER REACHED: ", corner_reached)
            # SM.setParam("corner_reached",cr)
        elif SM.getParam('turn_count') == 1 and cr == True: #Do second Turn
            print("Doing 2nd Turn")
            SM.setParam('vel_ref', 0)
            SM.setState('PivotInPlace')
            SM.setParam('turn_count', 2)
            SM.setParam('theta_d_gotten', False)
            # SM.setParam("corner_reached", cr)
        elif SM.getParam('turn_count') == 2 and cr == False: #Drive along outside
            print("looking for 2nd Corner")
            SM.setParam('vel_ref', 0.4)
            # SM.setParam("corner_reached", cr)
            print("CORNER REACHED: ", cr)
        elif SM.getParam('turn_count') == 2 and cr == True:# pivot in place at end of box
            print("doing 3rd Turn")
            SM.setParam('vel_ref', 0)
            SM.setState('PivotInPlace')
            SM.setParam('turn_count',3)
            SM.setParam('theta_d_gotten', False)
        elif SM.getParam('turn_count') == 3:
            SM.setState('LineFollow')
            SM.setParam('vel_ref', line_follow_speed)
        

        
        
    elif state =="RegainLine":
        pass
    else : #i.e. halt
        pass


def tagDetectionCallback(data):
    """ 
    param: data: given detections object
    view first id object 
    
    return: set corresponding
    reference velocity
    """
    global SM
    vel_dict = {
        '1':0.1,
        '2':0.2,
        '3': 0
    }
    if (len(data.detections) >= 1):
        tag_id = data.detections[0].id[0]
        print(tag_id)
        if vel_dict[str(tag_id)] != SM.getParam('vel_ref'):
            SM.setParam('vel_ref',vel_dict[str(tag_id)])


def odomCallback(data):
    global SM
    SM.setParam('vel_current', data.twist.twist.linear.x)
    SM.setParam('x', data.pose.pose.position.x)
    SM.setParam('y', data.pose.pose.position.y)
    on = data.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([on.x, on.y, on.z, on.w])
    SM.setParam('theta', theta)
    # print('Theta: %0.2f'%(theta))


def ctrlSetter(state):
    global SM
    global LF_OMEGA_PID
    v = CRUISE_CTRL_PID.getCtrlEffort(SM.getParam('vel_ref'),SM.getParam('vel_current'))
    omega=0
    if state == "LineFollow":
        #linefollowing ctrl code
        # v = CRUISE_CTRL_PID.getCtrlEffort(SM.getParam('vel_ref'),SM.getParam('vel_current'))
        thetaLine = 0
        y = SM.getParam('horz_center')
        ang_vel = SM.getParam('vel_ang_ref')
        dz_buf=20
        if (y < 320-dz_buf):
            # turn left
            thetaLine = ang_vel
        elif (y > 320+dz_buf):
            #turn right
            thetaLine = -ang_vel
        omega = thetaLine*2*abs(320-y)/320
        # omega = thetaLine * (1 + abs(y - 320)/160)
        # omega = thetaLine
        # omega = LF_OMEGA_PID.getCtrlEffort(320,y)
        # if abs(omega)>0.5:
        #     omega=np.sign(omega)*0.5
        # print('Omega: %0.2f'%(omega))
        # print("HZCTR: %0.2f | Error: %0.2f | Effort: %0.2f"%(y,320-y,omega))

    elif state =="PivotInPlace":
        v=0
        turn_count = SM.getParam('turn_count')
        print("TURN No: ",turn_count)
        theta = SM.getParam('theta')
        if SM.getParam('theta_d_gotten')==False:
            if turn_count in [1,4]:
                # print('Now at %d turn'%turn_count)
                theta_d = np.pi/2 + theta
            elif turn_count in [2]:
                # print('Now at %d turn'%turn_count)
                theta_d = (-np.pi/2) + theta
            elif turn_count in [3]:
                # print('Now at %d turn'%turn_count)
                theta_d = (-np.radians(80)) + theta
            SM.setParam('theta_d',theta_d)
            SM.setParam('theta_d_gotten',True)
        theta_d = SM.getParam('theta_d')
        print(theta_d)
        error = theta_d-theta
        # error=np.arctan2(np.sin(error),np.cos(error))
        if abs(error) < 0.25:
            SM.setState('DriveUntilClear')
        ctrl = PIDController(2,0.01,0,0.1)
        omega=1*ctrl.getCtrlEffort(theta_d,theta)
        # print("PIVOT ERROR: ",error)
        
        

    elif state =="DriveUntilClear":
        omega=0
        corner_reached=SM.getParam('corner_reached')
        if corner_reached == False:
            pass
        else:
            pass
            # if SM.getParam('goal_gotten')==False:
            #     goal=[SM.getParam('x'),SM.getParam('y')+0.1]
            #     SM.setParam('goal',goal)
            #     SM.setParam('goal_gotten',True)
            # goal = SM.getParam('goal')
            # # print("goalie- ",goal)
            # print(SM.getParam('x'),SM.getParam('y'))
            # error = np.linalg.norm([goal[0] - SM.getParam('x'),goal[1] - SM.getParam('y')])
            # # print('error',error)
            # if abs(error) < 0.1:
            #     SM.setState('PivotInPlace')
            #     if (SM.getParam('turn_count') == 4):
            #         SM.setParam('turn_count', 1)
            #     SM.setParam('turn_count',SM.getParam('turn_count')+1)
            
        #get waypoint
        #g2g
        pass
    else : #i.e. halt
        v=0
        omega=0
    
    vel_pub = Twist()
    vel_pub.linear.x = v
    vel_pub.angular.z = omega
    # print(vel_pub)
    SM.setParam('vel_pub', vel_pub)


### Script
def main():

    global CRUISE_CTRL_PID 
    CRUISE_CTRL_PID = PIDController(0.1,0.001,0.03,0.1)
    global LF_OMEGA_PID 
    LF_OMEGA_PID = PIDController(0.001,0.0001,0,0.1)

    states=["LineFollow","PivotInPlace","DriveUntilClear","RegainLine","Halt"]
    default_state="LineFollow"
    # DEFAULTS
    # 'vel_ref':0.2
    # 'vel_ang_ref':0.4
    sm_variables = {
        'err_tol':0.1,
        'y':0,
        'theta':0,
        'vel_current':0,
        'vel_ref':0.3,
        'vel_ang_ref':1.5,
        'vel_pub':0,
        'obstacle_detected':False,
        'horz_center':320,
        'goals':[],
        'delta_y':0,
        'end_object': False,
        'parallel':False,
        'turn_count':0,
        'corner_reached':False,
        'theta_d_gotten':False,
        'theta_d':0,
        'goal':None,
        'goal_gotten':False,
    }
    global SM 
    SM = StateMachine(states, default_state=default_state,**sm_variables)

    # CruiseController=PIDController(1,0,0,0.1)
    # LineSeeker=PIDController(1,0,0,0.1)
    line_follow=Vault(obs_detect_threshold=0.2,CoM=[120,320])
    obs_avoid=Vault(corner=None)
    line_reacq=Vault(corner=None)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pos_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.init_node('avoidObstacle')
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Subscriber("/scan", LaserScan, lidarCallback)
    rospy.Subscriber("/lane", Int32MultiArray, cameraCallback)
    # rospy.Subscriber("/lane/compressed", Int32MultiArray, cameraCallback)
    # rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tagDetectionCallback)
    rate = rospy.Rate(10)
    set_initial_pose = True
    
    
    while not rospy.is_shutdown(): #and len(GOALS) > 0:
        if set_initial_pose:
            initial_pose = PoseWithCovarianceStamped()
            pos_pub.publish(initial_pose)
            set_initial_pose = False

        # if STATEMACHINE.state == 'STOP':
        #     STATEMACHINE.runStop()
        # elif STATEMACHINE.state == 'RUN':
        #     STATEMACHINE.runGo()
        state = SM.getState()
        # print("State: "+ state +" | X: %0.2f, | Y: %0.2f \n"%(SM.getParam('x'), SM.getParam('y')))
        ctrlSetter(state)

        vel_pub.publish(SM.getParam('vel_pub'))
        print(SM.getParam('vel_ref'))
        rate.sleep()
        

if __name__=='__main__':
    main()



