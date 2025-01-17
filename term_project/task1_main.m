%% RUN RRT FUNCTION
clear all
clc

%number of joint angles in q_init and q_goal must match and number of joint
%angles will be equal to the number of links

%q_init = [0; 2*pi-.1; 0.1; 0.05];
%q_goal = [pi/2; pi/4; pi/4; 2*pi-0.3];
radius = 4; %rewiring radius
% q_init = [0; 2*pi-.1; 0.1; 0.05; 0.05];
% q_goal = [pi/2; pi/4; 2*pi - pi/4; 0; 3*pi/2]; 

q_init = [0; 2*pi-.1; 0.1];
q_goal = [pi/2; pi/4; 2*pi - pi/4]; 

O={[0 5 5 0; 10 10 15 15],[10 15 15 10; 3 3 7 7], [10 17 17 10; -10 -10 -5 -5],[-10 -15 -15 -10; 10 10 15 15]}; %obstacles in CCW order
r = 5; %this is the length of each link of the robot
dq = 1; %step sizes
max_iter = 1000; %maximum number of iterations
beta = 0.6; % weight factor balancing the two objectives (0-1)
path = RRT_star(q_init, q_goal, O, r, dq, radius, beta,max_iter); % pass beta to RRT_star

make_video(q_init, q_goal, O, r, path, 'RRT_star'); %function to use the path to make the video of robot in workspace