function [ pf ] = pf_correct( pf,z_meas )
    %% unloading struct
    particles = pf.particles;
    weights = pf.weights;
    ratio = pf.ratio;
    
%     meas_fcn = pf.meas_fcn;
    meas_likelihood = pf.meas_likelihood;
    meas_noise = pf.meas_noise;
    
    %% correction
    % finding new weights
%     z_pred = meas_fcn(particles);
    
    likelihood = meas_likelihood(particles, z_meas(:), meas_noise);
    c = sum(weights.*likelihood);
    weights = weights.*likelihood/c;
    
    [xhat,Phat] = pf_estimate(pf);
    
    % resampling
    [particles, weights] = sis_resample(particles, weights, ratio);
    
    %% updating struct
    pf.particles = particles;
    pf.weights = weights;
    
    pf.state = xhat;
    pf.state_covar = diag(Phat);
end

function [ x,w ] = sis_resample( x,w,ratio )
% function [ x,w ] = sis_resample( x,w )
%Function implementing sampling important sampling algorithm, i.e. sampling
%when number of effective samples decreases below a given threshold
%Inputs:
%       x:  particles to be checked
%       w:  weights
%Outputs:
%       x:  resampled particles
%       w:  resampled weights

    [~,N] = size(x);
    n_eff = 1/(sum(w.^2)); % approx for degree of depletion
    if(n_eff<N*ratio) % 2/3 or 1/2
%         fprintf('resample\n')
        [x,w] = sort_resample(x,w);
    end
end

function [ x,w ] = sort_resample( x,w )
%function [ x,w ] = sort_resample( x,w )
%Sampling using sort, optimized for vector programming languages [1]
%Inputs:
%       x:  particles to be resampled
%       w:  weights
%Outputs:
%       x:  resampled particles
%       w:  resampled weights
    [~,N] = size(x);
    % multinomial
    u = rand(1,N);
    % Stratified 
%     u=([0:N-1]+(rand(1,N)))/N; 
    % Systematic 
%     u=([0:N-1]+rand(1))/N;
    wc = cumsum(w);
    wc = wc/wc(N);
    [~,ind1] = sort([u, wc]);
    ind2 = find(ind1<=N);
    ind = ind2-(0:N-1);
    x = x(:,ind);
    w = ones(1,N)./N;
end

%  [1] F. Gustafsson, 
%       “Particle filter theory and practice with positioning applications,”
%       IEEE Aerospace and Electronic Systems Magazine, 
%       vol. 25, no. 7, pp.53–82, 2010.