# Warmup Project
*Anusha Karandikar*
*Introduction to Computational Robotics, Fall 2023*

* [Introduction](#introduction)
* [Behaviors](#behaviors)
    * [Robot Teleop](#teleop)
    * [Driving in a Square](#square)
    * [Wall Follower](#wall)
    * [Person Follower](#person)
    * [Obstacle Avoider](#obstacle)
    * [Finite State Controller](#controller)
* [Reflection](#reflection)

# Introduction
Over the course of this project, I implemented five primary behaviors on a Neato robot platform: teleoperation, driving in a square, wall following, person following, and obstacle avoidance. The final behavior was a finite state controller to switch between two behaviors (wall following and person following).

# Behaviors

## [Robot Teleop](/warmup_project/warmup_project/teleop.py)
The robot teleoperator controls the Neato’s movement depending on the user’s keyboard input. The W and X keys command the robot to drive forwards and backwards, respectively, at 1 meter per second. The S key brings it to a stop. I decided to have the A and D keys make the robot turn counterclockwise and clockwise, respectively, and then continue driving forward. For these turning commands, the robot turns at 1 radian per second for enough time to make a 90 degree turn, then drives straight per a linear velocity command. Once a key is pressed, the behavior continues until another key is pressed.

The code consists of one main if statement that checks for the keyboard input. If I had more time, I would want to structure my velocity commands more efficiently by using functions.

| Key    | Result |
| -------- | ------- |
| W | Drive forwards |
| A | Turn counterclockwise, then drive forwards |
| x | Drive backwards |
| D | Turn clockwise, then drive forwards |
| S | Stop |

## [Driving in a Square](/warmup_project/warmup_project/drive_square.py)
To make the robot drive in a square, I used a time-based approach to command the velocity messages. To calculate the time needed for the robot to drive the length of one side, I divided the desired length by the linear velocity I defined. Similarly, to find the time needed for the robot to turn a complete right angle, I divided the radians by the angular velocity. Once I had these time values, I used an if statement to check the time passed and call the functions defining the necessary velocities. Something I found tricky about this behavior was that I had to correct for turning time. Using the precise radians for a right angle turn resulted in a smaller turn in practice, so I experimentally increased the radians until the behavior looked correct.

If I had more time, I would have tried approaching the problem using the Neato's odometry.

## [Wall Follower](/warmup_project/warmup_project/wall_follower.py)
<p align="center">
    <img src="images/wall_follower.png">
    Figure 1: Wall follower conditions
</p>

The wall follower script pilots the Neato to drive forward along a wall while aligning itself to be parallel to the wall. It is assumed that the Neato is close to a wall on its left side when the program starts. The Neato platform returns LIDAR readings, which I used to determine its parallelism and distance to the wall. The main readings I used were for the angles of 45, 90, and 135 degrees.

The Neato defaults to purely driving forwards, with angular velocity controlled by the distance readings: if the distance reading from 45 degrees is *greater* than the reading from 135 degrees (leftmost diagram in Figure 1), this indicates that the Neato is oriented away from the wall. Therefore, the angular velocity is set to a positive (i.e., counterclockwise) value to turn the Neato back towards the wall. In this case, the angular velocity is *directly* proportionally controlled by the distance reading at 90 degrees. If the Neato has traveled far from the wall, it is imperative to correct the behavior before the Neato gets too far and no longer receives readings from the wall. Therefore, if the distance at 90 degrees is bigger, the Neato will turn back towards the wall faster, and vice versa; if the Neato is still close to the wall and the distance at 90 degrees is small, the Neato can turn more slowly.

Similarly, if the distance reading from 45 degrees is *less* than the reading from 135 degrees (rightmost diagram in Figure 1), this indicates that the Neato is oriented towards the wall, and risks running into it. Therefore, the angular velocity is set to a negative (i.e., clockwise) value to turn the Neato away from the wall. In this case, the angular velocity is *inversely* proportionally controlled by the distance reading at 90 degrees. If the Neato is oriented towards the wall, the heading of the robot needs to be changed quickly before it can run into the wall. Therefore, if the distance at 90 degrees is small, the angular velocity will be greater; if the distance is bigger, there is more time to correct the heading, so the angular velocity can be smaller.

If the distance readings from 45 and 135 degrees are *equal* (middle diagram in Figure 1), the robot tries to get parallel to the wall at a distance of 0.3 meters. If the distance reading at 90 degrees is above 0.3 meters, the angular velocity is set to a positive value to turn the Neato counterclockwise and get closer to the wall. If the distance reading at 90 degrees is less than 0.3 meters, the angular velocity is set to a negative value to turn the Neato clockwise and get further away from the wall.

For the wall follower, I chose to visualize the LIDAR scan readings at 45, 90, and 135 degrees. To do so, I converted the readings into cartesian coordinates using the following equations (with angle $A$) so that I could plot them in the Neato’s base_link coordinate frame:

<p align="center">
    $x = (\text{distance reading at } A&deg;) \times \cos(A&deg;)$
    $y = (\text{distance reading at } A&deg;) \times \sin(A&deg;)$
</p>

The code in this script is primarily structured in a series of nested if loops. For this behavior, like a few others, I monitored the bump sensors on the Neato to make it stop if they were active (i.e., if the Neato hit something). If I had more time, I would have explored condensing my code by containing the velocity commands in separate functions.

## [Person Follower](/warmup_project/warmup_project/person_follower.py)

<p align="center">
    <img src="images/person_follower.png">
    Figure 2: Person follower
</p>

## [Obstacle Avoider](/warmup_project/warmup_project/obstacle_avoider.py)

## [Finite State Controller](/warmup_project/warmup_project/finite_state_controller.py)

# Reflection
I chose to work alone so that I could work on every aspect of this project and build an intuition for robot programming. While I do think I gained what I wanted from the experience, completing the entire project by myself - especially with a lack of experience - took much longer than the project was supposed to.

The biggest hurdle I had to overcome to complete this project was my lack of experience with python and ROS. Upon reflection, I think I am much more comfortable with the basics of programming now, though there are a lot of advanced techniques I would like to learn to make my code better. I am aware that a lot of my code can be much more efficiently programmed than it is - unfortunately, I ran out of time to do so.