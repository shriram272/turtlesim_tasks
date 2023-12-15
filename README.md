Package name is cv_basics inside there are files for each task


Goal 1 - Control Turtle
for moving to goal position with pid control , after launching turtlesim run the turt.py file
[Screencast from 12-15-2023 05:25:13 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/2bcc4bfd-a2f1-4278-8c92-41c003d74bfd)


Goal 2- Make a grid
for making a proper grid preferably start from lower corner to avoid extra lines
run the decel.py file for making a grid . Code also includes pid control and deceleration profile to limit maximum acceleration

[Screencast from 12-15-2023 05:28:42 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/d5060cff-97e9-4679-8103-68f750be1f24)


Goal 3 - Rotate turtle in circle
for making a circle run the circle.py file
It has parameters to control linear/angular velocity and radius of circle along with pid control and deceleration profile
[Screencast from 12-15-2023 05:32:00 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/f1e0eeb9-51b6-4013-9fd6-abef7bb673db)
 for getting relative and noisy pose use-
 ros2 topic echo /rt_real_pose for real pose 
 ros2 topic echo /rt_noisy_pose for noisy pose with standard deviation of 10 units
 [Screencast from 12-15-2023 05:47:05 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/2d5960b3-b7f1-4b9a-aa60-eef742a3cf56)


 Goal 4 - Chase turtle
 spawn a new turtle with name turtle2 to chase the turtle moving in circle and stop when minimum distance is 1.5 units
 spawn turtle2
 ros2 run cv_basics police  -- to initiate chasing
 [Screencast from 12-15-2023 06:02:49 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/9c451a72-fe61-4bb5-bb89-2be6fc25c568)


 Goal 5 - Chase turtle slow
 to chase the turtle when the second turtle has lower velocity- The prediction logic is essentially a form of dead reckoning. It assumes that the target will continue moving in the same direction and at the same speed as it has been, 
 scaling this movement into the future based on the time_ahead parameter.
 parameter require more turing for smoother functioning. More complex methond like pure pursuit or other planning algorithms or behavior conditions for complex path following
 ros2 run cv_basics police2  -- to start chasing 

[Screencast from 12-15-2023 06:10:33 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/1b5ecd85-a278-4a27-b993-97bcc354b4bb)

Goal 6- Chase turtle slow with noisy pose
to chase turtle when first turtle is publishing noisy pose data
used moving average filter for smoothing out noisy pose data then used the same predictive logic
These filters smooth out the incoming noisy data by averaging the last n values (where n is the window size of the filter).
Kalman filter can be used for more proper functioning

[Screencast from 12-15-2023 06:32:52 PM.webm](https://github.com/shriram272/turtlesim_tasks/assets/99411053/c5a595b9-1016-4f55-a158-8d6cb851c8c8)


 
