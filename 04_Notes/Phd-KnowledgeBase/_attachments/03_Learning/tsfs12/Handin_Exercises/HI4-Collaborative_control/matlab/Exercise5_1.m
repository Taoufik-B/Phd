close all
clear all

addpath functions

% You need to follow the functions f1, h1, g1 to define your f2, h2, g2
% functions here

% Define functions f2 according to the description of Exercise 5.1 and use
% here
agent(1).f = @f1;
agent(2).f = @f1;
agent(3).f = @f1;
agent(4).f = @f1;

% Think about how to update the model parameters here, as you need to
% include wind force
modelparam.m = 1;

agent(1).mdlpar = modelparam;
agent(2).mdlpar = modelparam;
agent(3).mdlpar = modelparam;
agent(4).mdlpar = modelparam;

% Define functions g2 according to the description of Exercise 5.1 and use
% here
agent(1).g = @g1;
agent(2).g = @g1;
agent(3).g = @g1;
agent(4).g = @g1;

% Think about how to update this according to f2, g2, h2
controlparam1.k = 1;
controlparam2.k = 10;

agent(1).ctrlpar = controlparam1;
agent(2).ctrlpar = controlparam1;
agent(3).ctrlpar = controlparam1;
agent(4).ctrlpar = controlparam1;

% Define functions h2 according to the description of Exercise 5.1 and use
% here
agent(1).h = @h1;
agent(2).h = @h1;
agent(3).h = @h1;
agent(4).h = @h1;

% Think about how to update this according to f2, g2, h2
agent(1).measpar.meas_idx = 1:2;
agent(2).measpar.meas_idx = 3:4;
agent(3).measpar.meas_idx = 5:6;
agent(4).measpar.meas_idx = 7:8;

% Think about how to update this according to f2, g2, h2
agent(1).xref = @(t) [cos(2*t) sin(2*t)]'; 
agent(2).xref = @(t) [2*cos(-t) 2*sin(-t)]'; 
agent(3).xref = @(t) [3 2]'; 
agent(4).xref = @(t) [5 4]'; 


% SIMULATION

n=4; %number of states for each agent, you have 4 states here

% Think about how to update this according to f2, g2, h2
x0 = [1 0 2 0 3 0 4 0]'; % Initial values of the state vector.
tspan = 0:0.05:10; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
% You need to update the plot function to plot trajectory
for i=1:length(t)
    plot(x(i,1),x(i,2),'x',x(i,3),x(i,4),'o',x(i,5),x(i,6),'s',x(i,7),x(i,8),'d')
    axis([-2 6 -2 6])
    pause(0.03)
end


