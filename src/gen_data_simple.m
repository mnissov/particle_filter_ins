%% initializing necessary constants
if ~exist('Ts')
    Ts = [0.01; 0.1];
end
% defining vehicle acceleration
acc = 0.1*[
    ones(0.1*Tf/Ts(1),1);    zeros(0.8*Tf/Ts(1),1);   -ones(0.1*Tf/Ts(1),1)
];
% defining vehicle ang velocities
angvel = zeros(size(acc));

%% create data and add noise
% create data struct
[imu, gnss] = gen_ins_data(acc, angvel, Ts(1), Ts(2), 'tangent');

% adding noise
noise_param