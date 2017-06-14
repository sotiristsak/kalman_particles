% clear everything
clear all
close all
clc
% rng(1541)
global  control
global  radar

% Load data
control = load('dataset/control.dat');
radar = load('dataset/radar1.dat');

results = zeros(20,7)

% Initial state
x=[5.1;4.1996;-0.5;0.3]'; 

% seed=10;
% [max,index] = particle_fun_withoutPlot(seed,x)
counter = 1
for seed = 10:10:200
    
    [max,index] = particle_fun_withoutPlot(seed,x);
    results(counter,:) = [x seed max index];
    counter = counter + 1
end

% max(results(:,6))