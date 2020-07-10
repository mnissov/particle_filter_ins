if ~exist('add_bias')
    add_bias = 0;
end
if add_bias == 1
%     b_a = 0.05*ones(3,1);
%     b_g = 3.5e-4*ones(3,1);
    b_a = repmat(0.05, 3, length(imu.t));
    b_g = repmat(3.5e-4, 3, length(imu.t));
elseif add_bias == 2
    [~,b_a] = ode45(@(t,x) -1/1e4*x+1e-3*randn(1,1),...
        imu.t, ones(1,1)); % 5e-2
    b_a = repmat(b_a, 1, 3)';
%     [~,b_g] = ode45(@(t,x) -1/1e4*x+1e-5*randn(1,1),...
%         imu.t, ones(1,1)); % 5e-3
%     b_g = repmat(b_g, 1, 3)';
    b_g = zeros(size(b_a));
elseif add_bias == 3
    b_a = repmat(0.05, 3, length(imu.t));
    b_g = repmat(0, 3, length(imu.t));
else
    b_a = zeros(3,length(imu.t));
    b_g = zeros(3,length(imu.t));
end
if exist('add_noise')
    if add_noise==1
        % dimension noise matrices
        R_acc = 1e-4*eye(3)/Ts(1);
        R_gyro = 1e-5*eye(3)/Ts(1);
        R_gnss = 10*eye(3)/Ts(2);
        [imu, gnss] = ins_add_noise(imu, gnss,...
            R_acc, R_gyro, R_gnss,...
            b_a, b_g);
    elseif add_noise==2
        % dimension noise matrices
        R_acc = 60e-6/9.80665*sqrt(375)*eye(3);
        R_gyro = 0.01*sqrt(415)*(pi/180)^2*eye(3);
        R_gnss = 5^2*eye(3);
        [imu, gnss] = ins_add_noise(imu, gnss,...
            R_acc, R_gyro, R_gnss,...
            b_a, b_g);
    elseif add_noise == 3
        % dimension noise matrices
        R_acc = 60e-6/9.80665*sqrt(375)*eye(3);
        R_gyro = 0.01*sqrt(415)*(pi/180)^2*eye(3);
        R_gnss = 5^2*eye(3);
        [imu, gnss] = ins_add_noise(imu, gnss,...
            R_acc, [], R_gnss,...
            b_a, b_g);
    else
        R_acc = 60e-6/9.80665*sqrt(375)*eye(3);
        R_gyro = 0.01*sqrt(415)*(pi/180)^2*eye(3);
        R_gnss = 5^2*eye(3);
        [imu, gnss] = ins_add_noise(imu, gnss,...
            [], [], [],...
            b_a, b_g);
    end
end