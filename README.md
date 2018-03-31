# Introduction
This is my first flight simulator project in relation to my online course about autonomous flying system at Udacity's Flying Car Nanodegree. Please see the original project instruction and its quick setup in details in `instruction.md`.

In short, this project allows me to program the flying path or waypoints of a drone in the simulator in order to make the drone flies autonomously. I wrote the code in Python and applied the _event driven programming_ concept. The goal of this project is to program the drone so that it flies autonomously in a particular square waypoints.

# Writeup
The initial code provides 6 states as an enumeration and its transitions. These steps describe my approach to finish this project:

1. Understand the transition functions and the callback concept
2. Implement the functionality until the drone can takeoff
3. Implement the forward flying
4. Implement the square waypoints 
5. Create a state machine, like this:
![State Machine][sm_img]
6. Testing and bug fixing
7. Integrate `visdom` for real-time plotting

# Result
As the result, I present the pose, altitute plot, and the video:

| Pose | Altitute |
|:--:|:--:|
|![alt text][pose_img]|![alt text][alt_img]|

![Result video][res_vid]

<video src="./imgs/result.webm" width="960">
</video>

[//]: # (References)
[pose_img]: ./imgs/pose_plot.png
[alt_img]: ./imgs/alt_plot.png
[sm_img]: ./imgs/statemachine.png
[res_vid]: ./imgs/result.webm