close all
clear all

idx = {1:4, 5:8, 9:12, 13:16, 17:20, 21:24, 25:28, 29:32};

agent(1).measpar.meas_idx = idx{1};
agent(2).measpar.meas_idx = cat(3,[idx{2};idx{1}],[idx{2};idx{3}]);
agent(3).measpar.meas_idx = cat(3,[idx{3};idx{2}],[idx{3};idx{4}]);
agent(4).measpar.meas_idx = cat(3,[idx{4};idx{3}],[idx{4};idx{5}]);
agent(5).measpar.meas_idx = cat(3,[idx{5};idx{4}],[idx{5};idx{6}]);
agent(6).measpar.meas_idx = cat(3,[idx{6};idx{5}],[idx{6};idx{7}]);
agent(7).measpar.meas_idx = cat(3,[idx{7};idx{6}],[idx{7};idx{8}]);
agent(8).measpar.meas_idx = cat(3,[idx{8};idx{7}]);


agent(1).xref = @(t) [0 4 0 0]';
agent(2).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(3).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(4).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(5).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(6).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(7).xref = @(t) [0 0;1 -1; 0 0;0 0];
agent(8).xref = @(t) [0 1 0 0]';

% Firstly create your state, measurememt, and control functions, and assign
% them correctly to the corresponding agents. Note you have two types of
% agents here.

% Then the implementation is similar to what you did in Exercise 5.1

