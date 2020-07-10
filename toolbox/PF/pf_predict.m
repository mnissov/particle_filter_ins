function [ pf ] = pf_predict( pf,u )
    particles = pf.particles;
    state_fcn = pf.state_fcn;
%     process_noise = pf.process_noise;
    
%     dither = process_noise*randn(size(particles));
    new_particles = state_fcn(particles, u(:));
    
    pf.particles = new_particles;
end