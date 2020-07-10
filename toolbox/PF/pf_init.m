function [ pf ] = pf_init( state_transition,meas_likelihood,...
    num_particles,x_init,P_init,dist_type,meas_noise )
    [nx,~] = size(x_init);
    
    %% creating normally distributed particle cloud
    sigma = chol(P_init, 'lower');
    if dist_type == "normal"
        particles = x_init + sigma*randn(nx,num_particles);
        weights = ones(1,num_particles)/num_particles;
    elseif dist_type == "uniform"
        a = x_init - sqrt(3)*diag(sigma);
        b = x_init + sqrt(3)*diag(sigma);
        
        particles = a + (b-a).*rand(nx,num_particles);
        weights = ones(1,num_particles)/num_particles;
    else
        error('Choose a valid distribution');
    end

    %% assigning struct
    % functions
    pf.state_fcn = state_transition;
    
%     pf.meas_fcn = meas_fcn;
    pf.meas_likelihood = meas_likelihood;
    % measurement noise
%     pf.process_noise = process_noise;
    pf.meas_noise = meas_noise;

    % particle cloud
    pf.num_particles = num_particles;
    pf.particles = particles;
    pf.weights = weights;
    
    pf.ratio = 2/3;
    
    % intial state and covariance
    pf.state = x_init;
    pf.state_covar = P_init;
end