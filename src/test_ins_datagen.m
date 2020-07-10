close all; clear; clc
%% Testing INS data gen function
addpath(genpath('../toolbox'))
Tf = 200;
turn_rate = 2.5;

add_noise = 2;
gen_data_complex

%% plotting - no noise
opts = {'interpreter','latex','fontsize',14};
xtext = 'Time $[s]$';

figure(1)
plot(imu.t, acc,...
    'linewidth',3)
grid on
xlabel('Time $[s]$',...
    opts{:})
ylabel('Specific Thrust $[m/s^2]$',...
    opts{:})
ylim([min(acc)*1.1, max(acc)*1.1])

figure(2)
plot(imu.t, rad2deg(imu.angvel(:,3)),...
    'linewidth',3)
grid on
xlabel(xtext,...
    opts{:})
ylabel('Yaw Rate $[deg/s]$',...
    opts{:})
ylim([min(-eps+rad2deg(imu.angvel(:,3)))*1.1,...
    max(eps+rad2deg(imu.angvel(:,3)))*1.1])

figure(3)
clf
subplot(321)
hold on
plot(imu.t, imu.acc(:,1),...
    'linewidth',2)
% plot(imu.t, imu.meas(:,1),...
%     '--','linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$a_x^b$ $[m/s^2]$',...
    opts{:})
subplot(322)
hold on
plot(imu.t, imu.acc(:,2),...
    'linewidth',2)
% plot(imu.t, imu.meas(:,2), '--')
grid on
xlabel(xtext,...
    opts{:})
ylabel('$a_y^b$ $[m/s^2]$',...
    opts{:})
subplot(3,2,3)
plot(imu.t, imu.vel(:,1),...
    'linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$v_n^t$ $[m/s]$',...
    opts{:})
subplot(3,2,4)
plot(imu.t, imu.vel(:,2),...
    'linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$v_e^t$ $[m/s]$',...
    opts{:})
subplot(3,2,[5])
hold on
plot(imu.t, imu.pos(:,1),...
    'linewidth',2)
% plot(gnss.t, gnss.meas(:,1), 'mx', 'markersize',8)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$p_n$ $[m]$',...
    opts{:})
subplot(3,2,[6])
hold on
plot(imu.t, imu.pos(:,2),...
    'linewidth',2)
% plot(gnss.t, gnss.meas(:,2), 'mx', 'markersize',8)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$p_e$ $[m]$',...
    opts{:})

figure(4)
clf
subplot(211)
hold on
plot(imu.t, rad2deg(imu.angvel(:,3)),...
    'linewidth',2)
% plot(imu.t, rad2deg(imu.meas(:,6)),...
%     '--')
grid on
xlabel(xtext,...
    opts{:})
ylabel('$\omega_\psi$ $[deg/s]$',...
    opts{:})
subplot(212)
plot(imu.t, rad2deg(imu.yaw),...
    'linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$\psi$ $[deg]$',...
    opts{:})

%% plotting - w/ noise
figure(5)
clf
subplot(421)
hold on
plot(imu.t, imu.acc(:,1),...
    'linewidth',2)
plot(imu.t, imu.meas(:,1),...
    '--','linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$a_x^b$ $[m/s^2]$',...
    opts{:})
subplot(422)
hold on
plot(imu.t, imu.acc(:,2),...
    'linewidth',2)
plot(imu.t, imu.meas(:,2),...
    '--','linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$a_y^b$ $[m/s^2]$',...
    opts{:})
subplot(4,2,[3,4])
hold on
plot(imu.t, rad2deg(imu.angvel(:,3)),...
    'linewidth',2)
plot(imu.t, rad2deg(imu.meas(:,6)),...
    '--','linewidth',2)
grid on
xlabel(xtext,...
    opts{:})
ylabel('$\dot{\psi}_{tb}^b$ $[deg/s]$',...
    opts{:})
subplot(4,2,[5,6,7,8])
hold on
plot(imu.pos(:,2), imu.pos(:,1),...
    'linewidth',3)
plot(gnss.meas(:,2), gnss.meas(:,1),...
    'x','linewidth',1,'markersize',8)
grid on
xlabel('$p_e$ $[m]$',...
    opts{:})
ylabel('$p_n$ $[m]$',...
    opts{:})
xlim([-10, max(imu.pos(:,2))*1.05])
ylim([-10, max(imu.pos(:,1))*1.05])
% axis equal

%% checking validity
vel = diff(imu.pos)/imu.Ts;
acc_t = diff(vel)/imu.Ts;
acc_b = zeros(size(acc_t));
for k=1:length(acc_b)
    acc_b(k,:) = (Rz(imu.yaw(k))'*acc_t(k,:)')';
end

figure(6)
clf
subplot(221)
plot(vel(:,1))
grid on
subplot(222)
plot(vel(:,2))
grid on
subplot(223)
plot(acc_b(:,1))
grid on
subplot(224)
plot(acc_b(:,2))
grid on