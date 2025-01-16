%% RUN RRT FUNCTION
clear all
clc

%number of joint angles in q_init and q_goal must match and number of joint
%angles will be equal to the number of links

%q_init = [0; 2*pi-.1; 0.1; 0.05];
%q_goal = [pi/2; pi/4; pi/4; 2*pi-0.3];

q_init = [0; 2*pi-.1; 0.1; 0.05; 0.05];
q_goal = [pi/2; pi/4; 2*pi - pi/4; 0; 3*pi/2]; 
O={[0 5 5 0; 10 10 15 15],[10 15 15 10; 3 3 7 7], [10 17 17 10; -10 -10 -5 -5]}; %obstacles in CCW order
r = 5; %this is the length of each link of the robot
dq = 0.5; %step sizes

path = RRT(q_init, q_goal, O, r, dq); %run RRT based on input parameters
make_video(q_init, q_goal, O, r, path, 'RRT'); %function to use the path to make the video of robot in workspace


%% RUN PRM FUNCTION

%q_init = [0; 2*pi-.1; 0.1; 0.05];
%q_goal = [pi/2; pi/4; pi/4; 2*pi-0.3];

% q_init = [0; 2*pi-.1; 0.1; 0.05];
% q_goal = [pi/2; pi/4; 2*pi - pi/4; 1.2];
% n = 600; K = 20; 
% O={[0 5 5 0; 10 10 15 15],[10 15 15 10; 3 3 7 7], [10 17 17 10; -10 -10 -5 -5]};
% r = 5; %this is the length of each link of the robot

% path = PRM(q_init, q_goal, n, K, O, r); %run PRM based on input parameters
% make_video(q_init, q_goal, O, r, path, 'PRM'); %function to use the path to make the video of robot in workspace