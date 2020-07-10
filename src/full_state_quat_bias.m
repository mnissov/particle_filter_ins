function [ xdot ] = full_state_quat_bias( t,x,u )
%function [ xdot ] = full_state_quat_bias( t,x,u )
    
% unpacking
    [~,np] = size(x);
    % states
    pos = x(1:3,:);
    vel = x(4:6,:);
    quat = x(7:10,:);
    b_a = x(11:13,:);
    b_g = x(14:16,:);
    % inputs
    acc_b = u(1:3)-b_a;
    angvel = u(4:6)-b_g;
    
    R = quat2rotm(quat');
    acc_t = multiprod(R, acc_b, [1,2], [1]);
    
    % calculate derivatives wrt time
    pdt = pdot(vel);
    vdt = vdot(acc_t);
    qdt = qdot(quat, angvel);
    badt = bdot(b_a);
    bgdt = bdot(b_g);
    
    xdot = [pdt; vdt; qdt; badt; bgdt];
end

function [ out ] = pdot( v )
    out = v;
end
function [ out ] = vdot( a )
    out = a;
end
function [ out ] = qdot( q,angvel )
    [~,np] = size(q);
    
    w = [zeros(1,np); angvel];
    out = zeros(size(q));
    for i=1:np
        out(:,i) = 0.5*quatmult(w(:,i), q(:,i));
    end
end
function [ out ] = bdot( b )
    out = -1/1e4*b;
%     out = zeros(size(b));
end