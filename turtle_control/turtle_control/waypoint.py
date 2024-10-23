'''
Accept waypoints and moves turtlesim node through those waypoints with a linear x velocity and uses an angular z
to rotate the turtle towards the next waypoint. Also is able to reset the turtlesim and its field, and toggle the 
state of the turtle to either moving or stopped.

PUBLISHERS:
turtle1/cmd_vel Twist           :   Velocity of turtle
loop_metrics ErrorMetric        :   Error metrics of turtle trajectory; includes turtle loop counter, distance traveled
                                    by the turtle and error between distance travelled by turtle and the actual total
                                    distance between each waypoint

SUBSCRIBERS:
pose Pose                       :   Position parameters of the turtle; includes x, y, theta (turtle orientation)

SERVICES OFFERED:
toggle Empty                    :   Toggles state of turtle between MOVING and STOPPED
load Waypoints                  :   Loads waypoints for turtle trajectory

SERVICES CALLED:
reset Empty                     :   Resets turtlesim window and turtle location
turtle1/teleport_absolute          
TeleportAbsolute                :   Teleports the turtle to a particular location specified by a waypoint
turtle1/set_pen SetPen          :   Used to manipulate the trail of the turtle 

PARAMETERS:
frequency double                :   Frequency of timer clock that the node runs on
tolerance double                :   Tolerance of distance between turtle and waypoint when turtle reaches waypoint 
'''

'''Importing all required predictions'''
import rclpy
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from enum import Enum, auto
from turtlesim.srv import TeleportAbsolute, Spawn, Kill, SetPen
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from turtle_interfaces.srv import Waypoints
from turtle_interfaces.msg import ErrorMetric
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist



class State(Enum):
    '''
    Current state of the system.
    Determines what the main timer function should be doing on each iteration
    '''
    STOPPED = auto(),
    MOVING = auto(),

class WaypointNode(Node):
    '''
    Subscribes to position of turtle and publishes error metrics to turtle_interfaces. Creates clients for turtlesim 
    reset, teleporting the turtle, and setting pen of turtle trail. Creates services for toggling state of turtle and 
    loading waypoiints to node.
    '''
    def __init__(self):
        super().__init__('waypoint')
        self.state = State.STOPPED
        
        self.cb_group = ReentrantCallbackGroup() #  should generally prefer a MutuallyExclusive Callback Group: I don't see any callbacks here that you would want to be re-entered
        self.vel = Twist()              # velocity of turtle [3]
        self.waypoints = []             # array of waypoints 
        self.goal_theta = 0.0           # angle goal turtle needs to reach (differential drive)
        self.goal_wp = 0.0              # waypoint goal turtle needs to travel to
        self.no_wp = 0.0                # number ofwaypoints loaded
        self.curr_wp = 0.0              # current waypoint turtle is located or traveling from
        self.actual_dist = 0.0          # actual distance between all waypoints
        self.metric = ErrorMetric()     # error metric object

        '''Declaring readable params '''
        ########################## Begin_Citation [1] ######################################
        self.declare_parameter('frequency', 90.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.declare_parameter('tolerance', 0.1)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        ########################## End_Citation [1] ########################################

        ''' Create services '''
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        self.load = self.create_service(Waypoints, "load", self.load_callback)

        ''' Create clients '''
        self.reset = self.create_client(Empty, "reset", callback_group=self.cb_group)
        self.tele_abs = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.cb_group) # [3]
        self.set_pen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.cb_group) # [3]

        ''' Create subscriber '''
        self.subscription = self.create_subscription(Pose, "turtle1/pose", self.subscriber_callback,10) # [3]
        self.pose = Pose
        self.prev_pose = None

        ''' Create publisher '''
        self.pub_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10) # [3]
        self.pub_metric = self.create_publisher(ErrorMetric, "loop_metrics", 10)

        ''' Create a timer with a callback '''
        timer_period = 1.0 / self.frequency  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def reset_metric(self):
        ''' Resets error metrics '''
        self.metric.complete_loops = 0
        self.metric.real_dist = 0.0
        self.metric.error = 0.0

    def timer_callback(self):
        '''
        Callback function for the timer

        Manipulates linear x component and angular z component of the velocity depending on state of turtle
        '''

        '''Turtle does not move in y dirn. to mimic differential drive robot'''
        self.vel.linear.y = 0.0

        if self.state == State.MOVING:
            '''
            If turtle's state is moving, turtle is moved towards the next waypoint 
            '''    
            self.get_logger().debug("Issuing Command!")
            '''Distance between turtle's current position and its goal waypoint is calculated'''
            dist_rem = self.get_remaining_distance(self.pose, self.goal_wp)
            '''Goal orientation of the turtle is calculated'''
            self.goal_theta = self.get_theta()
            if dist_rem <= self.tolerance:
                '''
                If remaining distance is below tolerance, turtle is deemed to reach waypoint and the next waypoint is set as the goal waypoint
                '''
                self.get_next_wp()
            else:
                '''
                If remaining distance is above tolerance, turtle needs to still be driven to goal waypoint
                '''
                ##################### Begin_Citation [2, turtle_control/waypoint.py, 325-338] #############################
                if abs(self.pose.theta - self.goal_theta) > 0.01: 
                    '''
                    If goal angle and turtle's orientation are not equal, drive turtle rotation in either direction to match the goal angle
                    '''
                    if self.pose.theta - self.goal_theta > 0:
                        self.vel.angular.z = -0.5
                        self.vel.linear.x = 0.0
                    else:
                        self.vel.angular.z = 0.5
                        self.vel.linear.x = 0.0
                else:
                    '''
                    If goal angle is equal to turtle's orientation, drive turtle  linear x velocity component towards waypoint
                    '''
                    self.vel.angular.z = 0.0
                    self.vel.linear.x = 7.0    
                ####################### End_Citation [2] #############################

        elif self.state == State.STOPPED:
            '''If turtle's state is stopped, change all velocity components to zero to actually stop the turtle'''
            self.reset_vel()

        '''Publishing velocity to turtle1/cmd_vel to drive or stop the turtle accordingly'''
        self.pub_vel.publish(self.vel)


    def get_remaining_distance(self, pose, wp):
        '''
        Function to get distance remaining between turtle's current position and its goal waypoint
        
        ARGS:
        pose: Current position of the turtle
        wp: Goal waypoint of the turtle
        
        RETURNS:
        dist: Remaining distance to goal waypoint
        '''
        x = pose.x
        y = pose.y
        ################### Begin_Citation [2, turtle_control/waypoint.py, 229] ################################
        dist = np.linalg.norm([x,y] - self.waypoints[wp])
        #################### End_Citation [2] #################################
        self.actual_dist += dist
        return dist
    

    def get_next_wp(self):
        '''Function to get next waypoint as goal waypoint when the original goal waypoint has reached'''
        self.curr_wp = self.goal_wp
        if self.goal_wp == self.no_wp-1:
            self.goal_wp = 0
        else:
            self.goal_wp += 1
        if self.curr_wp == 0 and self.metric.real_dist != 0.0:
            '''Calculating error metrics when a loop of waypoints is finished'''
            self.metric.complete_loops += 1
            self.metric.error = abs(self.metric.real_dist - self.actual_dist)
            self.pub_metric.publish(self.metric)

    def get_theta(self):
        '''
        Function to get goal angle needed to drive turtle to goal waypoint

        RETURNS:
        goal angle
        '''
        ############################ Begin_Citation [2, turtle_control/waypoint.py, 182-184] ###############################
        next_wpoint=self.waypoints[self.goal_wp]  
        curr_wpoint=[self.pose.x,self.pose.y]
        return np.arctan2([next_wpoint[1]-curr_wpoint[1]], [next_wpoint[0]-curr_wpoint[0]])
        ############################# End_Citation [2] #################################
    

    def toggle_callback(self, request, response):
        '''
        Callback function to trigger toggle service 

        Toggles state of turtle between STOPPED and MOVING

        ARGS:
        request - service request from user

        RETURNS:
        response 
        ''' 
        
        if self.state == State.STOPPED:
            if not self.waypoints:
                self.get_logger().info('ERROR: Load waypoints before toggle')
            else:
                self.state = State.MOVING 

        elif self.state == State.MOVING:
            self.state = State.STOPPED
            self.get_logger().info('Stopping')

        return response
    
    ######################### Begin_Citation [2, turtle_control/waypoint.py, 126-156] ####################################
    async def draw_x(self, x, y):
        '''
        Function to draw X at a waypoint
        
        ARGS:
        x : x-coordinate of waypoint
        y : y-coordinate of waypoint
        '''
        #pen up
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2, off=1))
        #take to lower left corner of the cross
        await self.tele_abs.call_async(TeleportAbsolute.Request(x=x-0.25,y=y-0.25))
        #pen down
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        
        #take to upper right corner of the cross
        await self.tele_abs.call_async(TeleportAbsolute.Request(x=x+0.25,y=y+0.25))
        #pen up
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=1))
        
        #take to lower right corner of the cross
        await self.tele_abs.call_async(TeleportAbsolute.Request(x=x+0.25,y=y-0.25))
        #pen down
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        
        #take to upper left corner of the cross
        await self.tele_abs.call_async(TeleportAbsolute.Request(x=x-0.25,y=y+0.25))
        
        #pen up
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=1))    
    ######################## End_Citation [2] ########################################
    
    async def load_callback(self, request, response):
        '''
        Asynct callback function to trigger load service
        
        Load waypoints to node
        
        ARGS:
        waypoints: Waypoint coordinates accepted from service call on user side
        
        RETUNS:
        response
        '''
        self.reset_vel()
        self.pub_vel.publish(self.vel)
        await self.reset.call_async(Empty.Request())

        ########################### Begin_Citation [2, turtle_control/waypoint.py, 249-251] ################################
        for point in request.waypoints:
            self.waypoints.append(np.array((point.x, point.y)))
            await self.draw_x(point.x,point.y) 
        ########################### End_Citation[2] ####################################

        self.no_wp = len(self.waypoints)

        '''Teleporting turtle to first waypoint and setting pen on for turtle movement'''
        await self.tele_abs.call_async(TeleportAbsolute.Request(x = request.waypoints[0].x, y = request.waypoints[0].y))
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        '''Resets error metrics'''
        self.reset_metric()

        self.actual_dist = 0.0
        self.state = State.STOPPED
        '''Setting current waypoint to the first waypoint and goal waypoint to second waypoint'''
        self.curr_wp = 0
        self.goal_wp = 1

        return response

    def subscriber_callback(self, msg):
        '''
        Callback function for pose subscription
        
        Gets live position parameters from turtles1
        
        ARGS:
        msg: Position parameters of turtle
        '''
        self.pose = msg
        ########################### Begin_Citation [5] ####################################
        if self.prev_pose is not None:
            '''Calculates distance travelled by turtle if turtle is not located at the first waypoint'''
            prevX = self.prev_pose.x
            prevY = self.prev_pose.y
            currX = self.pose.x
            currY = self.pose.y
            self.metric.real_dist += math.sqrt((prevX - currX)**2 + (prevY - currY)**2)
        ############################ End_Citation [5] ######################################

        self.prev_pose = self.pose

    def reset_vel(self):
        '''Setting velocity of turtle to zero'''
        self.vel.linear.x = 0.0
        self.vel.angular._z = 0.0
    

def main(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
