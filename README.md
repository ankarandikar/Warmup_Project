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

| Key    | Result |
| -------- | ------- |
| W | Drive forwards |
| A | Turn counterclockwise, then drive forwards |
| x | Drive backwards |
| D | Turn clockwise, then drive forwards |
| S | Stop |

## [Driving in a Square](/warmup_project/warmup_project/drive_square.py)
![Visualization](./wall_follower.svg)
<img src="./wall_follower.svg">

## [Wall Follower](/warmup_project/warmup_project/wall_follower.py)

## [Person Follower](/warmup_project/warmup_project/person_follower.py)

## [Obstacle Avoider](/warmup_project/warmup_project/obstacle_avoider.py)

## [Finite State Controller](/warmup_project/warmup_project/finite_state_controller.py)

# Reflection
I chose to work alone so that I could work on every aspect of this project and build an intuition for robot programming. While I do think I gained what I wanted from the experience, completing the entire project by myself - especially with a lack of experience - took much longer than the project was supposed to.

The biggest hurdle I had to overcome to complete this project was my lack of experience with python and ROS. Upon reflection, I think I am much more comfortable with the basics of programming now, though there are a lot of advanced techniques I would like to learn to make my code better. I am aware that a lot of my code can be much more efficiently programmed than it is - unfortunately, I ran out of time to do so.