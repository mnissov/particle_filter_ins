close all; clear; clc
%% Testing kinematic different equations
% specifically euhler integration order, pdot before vdot, vice-versa, and
% an ode solver solution
addpath(genpath('../toolbox'))

%% Gen data
Tf = 200;

gen_data_simple;

%% simulation

pV1 = zeros(numel(imu.t),3);
vV1 = zeros(numel(imu.t),3);

pV2 = zeros(numel(imu.t),3);

ode = 0;
pV3 = zeros(numel(imu.t),3);
vV3 = zeros(numel(imu.t),3);

for k=2:numel(imu.t)
    pV1(k,:) = pV1(k-1,:) + imu.Ts*vV1(k-1,:);
    vV1(k,:) = vV1(k-1,:) + imu.Ts*imu.acc(k-1,:);
    pV2(k,:) = pV2(k-1,:) + imu.Ts*vV1(k,:);
    
    if ode
        [~,temp] = ode45(@diffeq, [imu.t(k-1), imu.t(k)],...
            zeros(6,1), [], imu.acc(k-1,:)');
        pV3(k,:) = temp(end,1:3);
        vV3(k,:) = temp(end,4:end);
    end
end

%% plotting
Tf = 2;
figure(1)
clf
subplot(211)
hold on
plot(imu.t, imu.pos(:,1),...
    'o','markersize',8)
plot(imu.t, pV1(:,1),...
    '.','markersize',8)
plot(imu.t, pV2(:,1),...
    'x','markersize',8)
grid on
xlim([0, Tf])
ylabel('p_{north}')
subplot(212)
hold on
plot(imu.t, imu.vel(:,1),...
    'o','markersize',8)
plot(imu.t, vV1(:,1),...
    '.','markersize',8)
if ode
    plot(imu.t, vV3(:,1))
end
grid on
xlim([0, Tf])
ylabel('v_{north}')

figure(2)
clf
hold on
plot(imu.pos(1:Tf/imu.Ts,2), imu.pos(1:Tf/imu.Ts,1),...
    'o','markersize',8)
plot(pV1(1:Tf/imu.Ts,2), pV1(1:Tf/imu.Ts,1),...
    '.','markersize',8)
grid on

figure(3)
clf
subplot(211)
hold on
plot(imu.t, (imu.pos(:,1)-pV1(:,1)).^2)
plot(imu.t, (imu.pos(:,1)-pV2(:,1)).^2,'--')
if ode
    plot(imu.t, (imu.pos(:,1)-pV3(:,1)).^2)
    legend({'p before v','v before p','ode45'})
else
    legend({'p before v','v before p'})
end
grid on
subplot(212)
hold on
plot(imu.t, (imu.vel(:,1)-vV1(:,1)).^2)
if ode
    plot(imu.t, (imu.vel(:,1)-vV3(:,1)).^2)
    legend({'euhler','ode45'})
end
grid on

%% plotting to save
xtext = 'Time $[s]$';
opts = {'interpreter','latex','fontsize',14};

figure(4)
clf
subplot(211)
plot(imu.t, (imu.pos(:,1)-pV1(:,1)).^2,...
    'linewidth',2)
grid on
ylim([0,max((imu.pos(:,1)-pV1(:,1)).^2)*1.05])
xlabel(xtext,...
    opts{:})
ylabel('Squared Error for $p_n$',...
    opts{:})
subplot(212)
plot(imu.t, (imu.vel(:,1)-vV1(:,1)).^2,...
    'linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('Squared Error for $v_n^t$',...
    opts{:})

function [ xdot ] = diffeq( t,x,u )
%Input:
%       x = vel at time k-1
%       u = acc at time k
    vel = x(4:end);
    acc = u;
    
    xdot = zeros(6,1);
    xdot(1:3) = vel(:);
    xdot(4:6) = acc(:);
end