close all; clear; clc
%% Simulate contianer.m
% container.m from Fossen:
%   https://github.com/cybergalactic/MSS/blob/master/VESSELS/container.m
addpath(genpath('../toolbox'))

%% simulate - acc
Ts = 0.01;
Tf = 500;
t = 0:Ts:Tf;
M = numel(t);

x = [
    1;
    zeros(7,1);
    0*pi/180;
    160
];
ui = x(9:10);

xV = repmat(x', M, 1);

for k=2:M
    [x,~] = container(xV(k-1,:), ui);
    xV(k,:) = euler2(x', xV(k-1,:), Ts);
end

%% plotting
figure(1)
clf
plot(diff(xV(:,1))/Ts)
grid on

figure(2)
plot(xV(:,1))
grid on

%% simulate - yaw rate
Ts = 0.01;
Tf = 500;
t = 0:Ts:Tf;
M = numel(t);

x = [
    xV(end,1);
    zeros(7,1);
    10*pi/180;
    150;
];
ui = x(9:10);

xV = repmat(x', M, 1);

for k=2:M
    [x,~] = container(xV(k-1,:), ui);
    xV(k,:) = euler2(x', xV(k-1,:), Ts);
end

%% plotting
figure(2)
clf
plot(xV(:,3)*180/pi)
grid on