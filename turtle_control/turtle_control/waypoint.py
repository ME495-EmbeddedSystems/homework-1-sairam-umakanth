
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from enum import Enum, auto
from turtlesim.srv import TeleportAbsolute, Spawn, Kill, SetPen
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from turtle_interfaces.srv import Waypoints


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

        # Declare and read the frequency parameter (default 90.0 Hz)
        self.declare_parameter('frequency', 90.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.waypoints = []


        # Create services
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        self.load = self.create_service(Waypoints, "load", self.load_callback)

        # Create clients
        self.reset = self.create_client(Empty, "reset", callback_group=self.cb_group)
        self.tele_abs = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.cb_group)
        self.set_pen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.cb_group)

        # Create a timer with a callback
        # self.frequency = 90
        timer_period = 1.0 / self.frequency  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info('Test')
        if self.state == State.MOVING:
            self.get_logger().debug("Issuing Command!")
    
    def toggle_callback(self, request, response):
        # toggle_callback changes state of the node from moving to stopped or vice versa
        if self.state == State.STOPPED:
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
        
        # #pen up
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=1))    
    
    async def load_callback(self, request, response):
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2, off=0))
        await self.reset.call_async(Empty.Request())

        # Straight from homework solution code
        for point in request.waypoints:
            self.waypoints.append(np.array((point.x, point.y)))
            await self.draw_x(point.x,point.y) # Drawing X at each waypoint

        # Set 
        await self.tele_abs.call_async(TeleportAbsolute.Request(x = request.waypoints[0].x, y = request.waypoints[0].y))
        await self.set_pen.call_async(SetPen.Request(r=255,g=255,b=255,width=2,off=0))

        return response


    





def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
