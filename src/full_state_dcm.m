function [ xdot ] = full_state_dcm( t,x,u )
%function [ xdot ] = full_state_dcm( t,x,u )
    
% unpacking
    [~,np] = size(x);
    % states
    pos = x(1:3,:);
    vel = x(4:6,:);
    R = reshape(x(7:15,:), 3, 3, []);
    % inputs
%     angvel = repmat(u(4:6),1,np);
    acc_b = u(1:3);
    angvel = u(4:6);
    
%     acc_t = zeros(3,np);
%     for k=1:np
%         acc_t(:,k) = R(:,:,k)*acc_b;
%     end
    acc_t = multiprod(R, acc_b, [1,2], [1]);
    
    % calculate derivatives wrt time
    pdt = pdot(vel);
    vdt = vdot(acc_t);
    Rdt = Rdot(R, angvel);
    
    xdot = [pdt; vdt; Rdt];
end

function [ out ] = pdot( v )
    out = v;
end
function [ out ] = vdot( a )
    out = a;
end
% function [ out ] = Rdot( R,w )
%     [~,~,np] = size(R);
%     out = zeros(size(R));
%     for i=1:np
%         out(:,:,i) = R(:,:,i)*skew(w);
%     end     
%     out = reshape(out, [], size(R,3));
% end
function [ out ] = Rdot( R,w )
    Rdot = multiprod(R, skew(w));
    out = reshape(Rdot, [], size(R,3));
end