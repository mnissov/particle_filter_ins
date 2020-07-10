function [ imu,gnss ] = gen_ins_data( acc,angvel_y,Ts_imu,Ts_gnss,frame )
%function [ imu,gnss ] = gen_ins_data( acc,yaw_rate,Ts_imu,Ts_gnss,Tf,frame )
    
    % init structs
    imu = struct;
    gnss = struct;
    
    % generate time vectors
    n_imu = numel(acc(:,1));
    Tf = n_imu*Ts_imu;
    t_imu = 0:Ts_imu:Tf-Ts_imu;
    
    t_gnss = 0:Ts_gnss:Tf-Ts_gnss;
    n_gnss = numel(t_gnss);
    
    % velocity and pos data in tangent frame
    % full acc in b frame
    if size(acc,2)==1
        acc_x = acc; acc_y = zeros(size(acc_x)); acc_z = zeros(size(acc_x));
        acc = [acc_x, acc_y, acc_z];
    end
    
    % full angvel in b frame
    angvel_r = zeros(size(acc_x)); angvel_p = zeros(size(acc_x));
    if size(angvel_y,1)<n_imu
        angvel_y = [angvel_y; zeros(n_imu-size(angvel_y,1),1)];
    end
    angvel = [angvel_r, angvel_p, angvel_y];
    
    % yaw wrt true north
    yaw = cumtrapz(t_imu, angvel_y);
    
    % vel in tangent frame
    vel_b = cumtrapz(t_imu, acc);
    for k=1:n_imu
        vel_t(k,:) = Rz(yaw(k))*vel_b(k,:)';
    end
    % diff vel to get measured acc
    acc_t = [diff(vel_t)/Ts_imu; (Rz(yaw(end))*acc(end,:)')'];
    acc_b = zeros(size(acc_t));
    for k=1:n_imu
        acc_b(k,:) = Rz(yaw(k))'*acc_t(k,:)';
    end
    
    pos_t = cumtrapz(t_imu, vel_t);
    gnss_pos = pos_t([1:Ts_gnss/Ts_imu:end],:);
    
    % IMU
    imu.n = n_imu;
    imu.Ts = Ts_imu;
    imu.t = t_imu;
    
    imu.acc = acc_b;
    imu.vel = vel_t;
    imu.pos = pos_t;
    imu.angvel = angvel;
    imu.yaw = yaw;
    
    imu.meas = [acc_b, angvel];
%     if isempty(R_acc)
%         imu.meas = acc;
%     else
%         imu.meas = acc + randn(n_imu,3)*chol(R_acc,'lower');
%     end
%     if isempty(R_gyro)
%         imu.meas = [imu.meas, angvel];
%     else
%         imu.meas = [imu.meas, angvel+randn(n_imu,3)*chol(R_gyro,'lower')];
%     end
    
    % GNSS
    gnss.Ts = Ts_gnss;
    gnss.t = t_gnss;
    gnss.n = n_gnss;
    
    if frame=="tangent"
        gnss.pos = gnss_pos;
%         gnss.pos = resample(pos, t_imu, Ts_gnss);
%         if isempty(R_gnss)
%             gnss.meas = gnss_pos;
%         else
%             gnss.meas = gnss_pos + randn(n_gnss,3)*chol(R_gnss,'lower');
%         end
        gnss.meas = gnss_pos;
    end
end