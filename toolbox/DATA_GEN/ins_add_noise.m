function [ imu,gnss ] = ins_add_noise( imu,gnss,varargin )
    narginchk(2, 7);
    
    num_arg = size(varargin,2);
    
    R_acc = [];
    R_gyro = [];
    R_gnss = [];
    b_a = zeros(3,1);
    b_g = zeros(3,1);
    if num_arg==1
        R_acc = varargin{1};
    elseif num_arg==2
        R_acc = varargin{1};
        R_gyro = varargin{2};
    elseif num_arg==3
        R_acc = varargin{1};
        R_gyro = varargin{2};
        R_gnss = varargin{3};
    elseif num_arg==5
        R_acc = varargin{1};
        R_gyro = varargin{2};
        R_gnss = varargin{3};
        b_a = varargin{4};
        b_g = varargin{5};
    end
    
    if isempty(R_acc)
        imu.meas = imu.acc + b_a';
    else
        imu.meas = imu.acc + b_a' + randn(imu.n,3)*chol(R_acc,'lower');
    end
    imu.b_a = b_a';
    
    if isempty(R_gyro)
        imu.meas = [
            imu.meas, imu.angvel + b_g';
        ];
    else
        imu.meas = [
            imu.meas,...
            imu.angvel + b_g' + randn(imu.n,3)*chol(R_gyro,'lower')
        ];
    end
    imu.b_g = b_g';

    if isempty(R_gyro)
        gnss.meas = gnss.pos;
    else
        gnss.meas = gnss.pos + randn(gnss.n,3)*chol(R_gnss,'lower');
    end
end