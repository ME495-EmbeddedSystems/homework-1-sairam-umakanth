# ME495 Embedded Systems Homework 1
Author: Sairam Umakanth
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints '{"waypoints": [{x: 1.4, y: 1.6}, {x: 2.2, y: 9.4}, {x: 7.2, y: 6.1}, {x: 4.0, y: 2.6}, {x: 8.2, y: 1.  5}, {x: 4.1, y: 5.3}]}'  ` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty` starts and stops the turtle.
4. Here is a video of the turtle in action.
   [me495_hw1_demo.webm](https://github.com/user-attachments/assets/44497c85-a41e-4a5b-afe7-b2e8183705f5)
    The turtle stays stationary for a moment. It then roughly follows the direction of the first two movements if the waypoints existed but there is a noticeable difference in trajecory orienyation. Shortly after that, it makes contact with the wall and it stays hitting the wall for some time but goes up and down the wall with some rotations. It rotates a few times and then stops,  in contact with the wall. 
