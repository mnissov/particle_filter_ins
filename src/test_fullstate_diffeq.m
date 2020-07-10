close all; clear; clc
%% Testing full state diff eq
addpath(genpath('../toolbox'))

%% Gen data
Ts = [0.01; 1];
Tf = 200;
turn_rate = 2.5;

gen_data_complex

%% simulation
xV = zeros(numel(imu.t),15);
xV(1,:) = [
    zeros(3,1); % pos
    zeros(3,1); % vel
    reshape(Rz(0),9,1); % R
];

for k=2:numel(imu.t)
    xV(k,:) = state_fcn(xV(k-1,:)', imu.meas(k-1,:)', imu.Ts);
%     [~,temp] = ode45(@full_state, [imu.t(k), imu.t(k)+imu.Ts],...
%         xV(k-1,:)', [], imu.meas(k-1,:)');
%     xV(k,:) = temp(end,:);
end

%% plotting
Tf = 10;
figure(1)
clf
subplot(211)
hold on
plot(imu.pos(1:Tf/imu.Ts,2), imu.pos(1:Tf/imu.Ts,1),...
    'o','markersize',8)
plot(xV(1:Tf/imu.Ts,2), xV(1:Tf/imu.Ts,1),...
    '.','markersize',8)
grid on
xlabel('p_{east}')
ylabel('p_{north}')
subplot(212)
hold on
plot(imu.t, imu.vel(:,1),...
    'o','markersize',8)
plot(imu.t, xV(:,4),...
    '.','markersize',8)
grid on
xlim([0, Tf])
ylabel('v_{north}')

figure(2)
clf
subplot(221)
hold on
plot(imu.t, imu.pos(:,1),...
    'o','markersize',8)
plot(imu.t, xV(:,1),...
    '.','markersize',8)
grid on
% xlim([0, Tf])
ylabel('p_n')
legend('cumtrapz','Euler')
subplot(222)
hold on
plot(imu.t, imu.pos(:,2),...
    'o','markersize',8)
plot(imu.t, xV(:,2),...
    '.','markersize',8)
grid on
% xlim([0, Tf])
ylabel('p_e')
legend('cumtrapz','Euler')
subplot(223)
hold on
plot(imu.t, imu.vel(:,1),...
    'o','markersize',8)
plot(imu.t, xV(:,4),...
    '.','markersize',8)
grid on
% xlim([0, Tf])
ylabel('v_n^t')
legend('cumtrapz','Euler')
subplot(224)
hold on
plot(imu.t, imu.vel(:,2),...
    'o','markersize',8)
plot(imu.t, xV(:,5),...
    '.','markersize',8)
grid on
% xlim([0, Tf])
ylabel('v_e^t')
legend('cumtrapz','Euler')

figure(3)
clf
hold on
plot(imu.pos(:,2), imu.pos(:,1),...
    'linewidth',2)
plot(xV(:,2), xV(:,1),...
    '--','linewidth',2)
grid on

%% plotting to save
xtext = 'Time $[s]$';
opts = {'interpreter','latex','fontsize',14};

figure(4)
clf
subplot(221)
plot(imu.t, (imu.pos(:,1)-xV(:,1)).^2,...
    'linewidth',3)
grid on
ylim([0,max((imu.pos(:,1)-xV(:,1)).^2)*1.05])
xlabel(xtext,...
    opts{:})
ylabel('S.E. for $p_n$',...
    opts{:})
subplot(222)
plot(imu.t, (imu.pos(:,1)-xV(:,1)).^2,...
    'linewidth',3)
grid on
ylim([0,max((imu.pos(:,1)-xV(:,1)).^2)*1.05])
xlabel(xtext,...
    opts{:})
ylabel('S.E. for $p_e$',...
    opts{:})
subplot(223)
plot(imu.t, (imu.vel(:,1)-xV(:,4)).^2,...
    'linewidth',3)
grid on
xlabel(xtext,...
    opts{:})
ylabel('S.E. for $v_n^t$',...
    opts{:})
subplot(224)
plot(imu.t, (imu.vel(:,1)-xV(:,4)).^2,...
    'linewidth',3)
grid on
xlabel(xtext,...
    opts{:})
ylabel('S.E. for $v_e^t$',...
    opts{:})

function [ xk1 ] = state_fcn( x,u,Ts )
    xk1 = x + Ts*(full_state(0,x,u));
end