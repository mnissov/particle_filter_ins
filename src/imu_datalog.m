close all; clear; clc
%% Read imu datalog

data = tdfread('data/imu_1567612522440.log');
data = data.x_0_1567612522473_0_00x2E000000_00x2E000000_1640x2E206487_0x2D6;

%% plotting
% IMU:  systemtime (ms), gpstime (ms), gpsfixflag,  lat (deg), lon (deg),
% roll (deg), pitch (deg), yaw (deg), ax (-), ay (-), az (-), wx (-), wy (-), wz (-), mx (-), my (-), mz (-), temperature (-)
figure(1)
clf
subplot(211)
plot(data(:,4))
grid on
ylabel('lat')
subplot(212)
plot(data(:,5))
grid on
ylabel('long')

figure(2)
clf
subplot(311)
plot(data(:,6))
grid on
ylabel('roll')
subplot(312)
plot(data(:,7))
grid on
ylabel('pitch')
subplot(313)
plot(data(:,8))
grid on
ylabel('yaw')

figure(3)
clf
subplot(311)
plot(data(:,9))
grid on
ylabel('a_x')
subplot(312)
plot(data(:,10))
grid on
ylabel('a_y')
subplot(313)
plot(data(:,11))
grid on
ylabel('a_z')

figure(4)
clf
subplot(311)
plot(data(:,12))
grid on
ylabel('w_x')
subplot(312)
plot(data(:,13))
grid on
ylabel('w_y')
subplot(313)
plot(data(:,14))
grid on
ylabel('w_z')