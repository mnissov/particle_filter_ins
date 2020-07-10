%% initializing necessary constants
if ~exist('Ts')
    Ts = [0.01; 1];
end
if ~exist('turn_rate')
    turn_rate = 2.5;
end
% defining vehicle acceleration
acc = 0.1*[
    ones(0.1*Tf/Ts(1),1);    zeros(0.8*Tf/Ts(1),1);   -ones(0.1*Tf/Ts(1),1)
];
% defining vehicle ang velocities
turn_time = 90/turn_rate/Ts(1); % in samples
leftover_time = length(acc)-3*turn_time;
if leftover_time<0
    error('Turn rate is too low');
end
angvel = deg2rad(turn_rate)*[
    zeros(leftover_time/2,1);   
    ones(turn_time,1); 
    zeros(leftover_time/10,1); 
    ones(turn_time,1);
    zeros(leftover_time/10,1);
    -ones(turn_time,1);
];

%% create data and add noise
% create data struct
[imu, gnss] = gen_ins_data(acc, angvel, Ts(1), Ts(2), 'tangent');

% adding noise
noise_param