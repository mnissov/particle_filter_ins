function [ xdot ] = full_state_dcm_bias( t,x,u )
%function [ xdot ] = full_state_bias( t,x,u )
    
% unpacking
    [~,np] = size(x);
    % states
    pos = x(1:3,:);
    vel = x(4:6,:);
    R = reshape(x(7:15,:), 3, 3, []);
    b_a = x(16:18,:);
    b_g = x(19:21,:);
    % inputs
%     angvel = repmat(u(4:6),1,np);
    acc_b = u(1:3)-b_a;
    angvel = u(4:6)-b_g;
    
    acc_t = zeros(3,np);
    for k=1:np
        acc_t(:,k) = R(:,:,k)*acc_b(:,k);
    end
    % calculate derivatives wrt time
    pdt = pdot(vel);
    vdt = vdot(acc_t);
    Rdt = Rdot(R, angvel);
    badt = bdot(b_a);
    bgdt = bdot(b_g);
    
    xdot = [pdt; vdt; Rdt; badt; bgdt];
end
% function [ out ] = qdot( q,angvel )
%     q = quaternion(q');
%     w = quaternion([0; angvel]');
%     out = compact(0.5*w*q)';
% end

function [ out ] = pdot( v )
    out = v;
end
function [ out ] = vdot( a )
    out = a;
end
function [ out ] = Rdot( R,w )
    [~,~,np] = size(R);
    out = zeros(size(R));
    for i=1:np
        out(:,:,i) = R(:,:,i)*skew(w);
    end     
    out = reshape(out, [], size(R,3));
end
function [ out ] = bdot( b )
%     out = -1/1e4*b;
    out = zeros(size(b));
end