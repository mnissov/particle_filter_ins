close all; clear; clc
%% Verifying measurements are normally distributed
addpath(genpath('../toolbox'))
rng(42);

%% generate data
% artificial
Ts = [0.01; 1];
Tf = 100;
turn_rate = 5;

add_noise = 1;
gen_data_complex
sampling_ratio = gnss.Ts/imu.Ts;

%% particles
x0 = [
    zeros(3,1); % pos
    zeros(3,1); % vel
    reshape(Rz(0),9,1); % R
];
P0 = diag([
    1*ones(3,1); % pos
    1*ones(3,1); % vel
    1e-3*ones(9,1); % R
]);
[nx,~] = size(x0);

pf = pf_init(@(x,u) state_fcn(x,u,imu.Ts),...
    [],...
    10e3, x0, P0, 'normal',...
    []);

%% propogate points
xV = cell(numel(gnss.t), 1);
xV{1} = pf.particles;

for k=1:numel(gnss.t)
    xV{k} = pf.particles;
    figure(1)
    histogram(pf.particles(1,:))
%     title(strcat("iteration number ",num2str(k)))
    
    for l=1:sampling_ratio
        pf = pf_predict(pf, imu.meas(sampling_ratio*(k-1)+l,:));
%         predict(pf, imu.meas(sampling_ratio*(k-1)+l,:)');
    end
end

%% plotting
opts = {'interpreter','latex','fontsize',14};

figure(2)
subplot(311)
histogram(xV{end}(1,:))
grid on
ylabel('$p_n$',...
    opts{:})
subplot(312)
histogram(xV{end}(2,:))
grid on
ylabel('$p_e$',...
    opts{:})
subplot(313)
histogram(xV{end}(3,:))
grid on
ylabel('$p_d$',...
    opts{:})

%% dfittool
dfittool(xV{end}(1,:))
dfittool(xV{end}(2,:))

function [ xk1 ] = state_fcn( x,u,Ts )
    xk1 = x + Ts*full_state(0,x,u);
end