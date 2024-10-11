
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from enum import Enum, auto
from turtlesim.srv import TeleportAbsolute, Spawn, Kill, SetPen
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from turtle_interfaces.srv import Waypoints
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist



class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STOPPED = auto(),
    MOVING = auto(),

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint')
        self.state = State.STOPPED
        
        self.cb_group = ReentrantCallbackGroup()
        self.vel = Twist() # velocity of turtle
        self.waypoints = []
        self.goal_theta = 0.0
        self.goal_wp = 0.0
        self.no_wp = 0.0
        self.curr_wp = 0.0

        # Declaring readable params
        self.declare_parameter('frequency', 90.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.declare_parameter('tolerance', 0.1)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value

        # Create services
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        self.load = self.create_service(Waypoints, "load", self.load_callback)

        # Create clients
        self.reset = self.create_client(Empty, "reset", callback_group=self.cb_group)
        self.tele_abs = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.cb_group)
        self.set_pen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.cb_group)

        # Create subscriber
        self.subscription = self.create_subscription(Pose, "turtle1/pose", self.subscriber_callback,10)
        self.pose = Pose

        # Create publisher
        self.pub_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        # Create a timer with a callback
        # self.frequency = 90
        timer_period = 1.0 / self.frequency  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info('Test')
        self.vel.linear.y = 0.0

        if self.state == State.MOVING:

            self.get_logger().debug("Issuing Command!")
            dist_rem = self.get_remaining_distance(self.pose, self.goal_wp)
            self.goal_theta = self.get_theta()
            if dist_rem <= self.tolerance:
                self.get_next_wp()
            else:
                if abs(self.pose.theta - self.goal_theta) > 0.01:
                    if self.pose.theta - self.goal_theta > 0:
                        self.vel.angular.z = -0.5
                        self.vel.linear.x = 0.0
                    else:
                        self.vel.angular.z = 0.5
                        self.vel.linear.x = 0.0
                else:
                    self.vel.angular.z = 0.0
                    self.vel.linear.x = 7.0    

        elif self.state == State.STOPPED:
            self.reset_vel()

        self.pub_vel.publish(self.vel)


    def get_remaining_distance(self, pose, wp):
        x = pose.x
        y = pose.y
        return np.linalg.norm([x,y] - self.waypoints[wp])
    

    def get_next_wp(self):
        self.curr_wp = self.goal_wp
        if self.goal_wp == self.no_wp-1:
            self.goal_wp = 0
        else:
            self.goal_wp += 1

    def get_theta(self):
        next_wpoint=self.waypoints[self.goal_wp]  
        curr_wpoint=[self.pose.x,self.pose.y]
        return np.arctan2([next_wpoint[1]-curr_wpoint[1]], [next_wpoint[0]-curr_wpoint[0]])
    

    def toggle_callback(self, request, response):
        # toggle_callback changes state of the node from moving to stopped or vice versa
        if self.state == State.STOPPED:
            if not self.waypoints:
                self.get_logger().info('ERROR: Load waypoints before toggle')
            else:
                self.state = State.MOVING 

        elif self.state == State.MOVING:
            self.state = State.STOPPED
            self.get_logger().info('Stopping')

        return response
    
    async def draw_x(self, x, y):
        # also taken straight from solution code    
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
    
    async def load_callback(self, request, response):

        self.reset_vel()
        self.pub_vel.publish(self.vel)
        await self.reset.call_async(Empty.Request())

        # Straight from homework solution code
        for point in request.waypoints:
            self.waypoints.append(np.array((point.x, point.y)))
            await self.draw_x(point.x,point.y) # Drawing X at each waypoint

        self.no_wp = len(self.waypoints)

        # Set turtle to first waypoint location
        await self.tele_abs.call_async(TeleportAbsolute.Request(x = request.waypoints[0].x, y = request.waypoints[0].y))
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))
        self.state = State.STOPPED

        self.curr_wp = 0
        self.goal_wp = 1

        return response

    def subscriber_callback(self, msg):
        self.pose = msg

    def reset_vel(self):
        self.vel.linear.x = 0.0
        self.vel.angular._z = 0.0
    


    





def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
