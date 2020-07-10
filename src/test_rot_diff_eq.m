close all; clear; clc
%% test rotation diff eq
%% wrt DCM
R = Rz(0);
[T,Y] = ode45(@Rdot, [0:0.1:9],...
    R, [], [0; 0; deg2rad(10)]);

reshape(Y(end,:), 3, 3)

Ts = 0.01;
for k=0:Ts:9
    R = R+Ts*reshape(Rdot(0, R, [0;0;deg2rad(10)]), 3,3);
end
R

%% wrt quaternions
q = [1;0;0;0];
[T,Y] = ode45(@qdot, [0:0.1:9],...
    q, [], [0; 0; deg2rad(10)]);

quat2rotm(Y(end,:))

Ts = 0.01;
for k=0:Ts:9
    q = q+Ts*qdot(0, q, [0;0;deg2rad(10)]);
end
quat2rotm(q')

%% for the full state diff eqs
x0 = [
    zeros(3,1)
    zeros(3,1)
    reshape(Rz(0), 9, 1);
];
[T,Y] = ode45(@full_state, [0:0.1:9],...
    x0, [], [zeros(3,1); 0; 0; deg2rad(10)]);
reshape(Y(end,7:15), 3, 3)

Ts = 0.01;
for k=0:Ts:9
    x0 = x0+Ts*full_state(0, x0, [zeros(3,1);0;0;deg2rad(10)]);
end
reshape(x0(7:15), 3, 3)

function [ out ] = Rdot( t,R,w )
    R = reshape(R, 3, 3);
    out = R*skew(w);
    out = out(:);
end
function [ out ] = qdot( t,q,angvel )
    q = quaternion(q');
    w = quaternion([0; angvel]');
    out = compact(0.5*w*q)';
end