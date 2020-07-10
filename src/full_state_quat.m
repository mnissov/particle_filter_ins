function [ xdot ] = full_state_quat( t,x,u )
%function [ xdot ] = full_state( t,x,u )
    
% unpacking
    [~,np] = size(x);
    % states
    pos = x(1:3,:);
    vel = x(4:6,:);
    q = x(7:10,:);
    % inputs
%     angvel = repmat(u(4:6),1,np);
    acc_b = u(1:3);
    angvel = u(4:6);
    
    R = quat2rotm(q');
    acc_t = multiprod(R, acc_b, [1,2], [1]);
    
    % calculate derivatives wrt time
    pdt = pdot(vel);
    vdt = vdot(acc_t);
    qdt = qdot(q, angvel);
    
    xdot = [pdt; vdt; qdt];
end

function [ out ] = pdot( v )
    out = v;
end
function [ out ] = vdot( a )
    out = a;
end
function [ out ] = qdot( q,angvel )
    [~,np] = size(q);
    
    w = [0; angvel];
    out = zeros(size(q));
    for i=1:np
        out(:,i) = 0.5*quatmult(w, q(:,i));
    end
end