function [ xhat,Phat ] = pf_estimate( pf )
    particles = pf.particles;
    weights = pf.weights;
    
    xhat = sum(particles.*weights,2);
    Phat = sum((particles-xhat).^2.*weights, 2);
end