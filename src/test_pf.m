close all; clear; clc
%% Demonstrating particle filter
addpath(genpath('../toolbox'))

%% gathering relevant data
addpath(fullfile(matlabroot,'examples','control','main')) % add example data

load ('vdpODEdata.mat','xTrue','dt')
tSpan = 0:dt:5;
sqrtR = 0.04;
yMeas = xTrue(:,1) + sqrtR*randn(numel(tSpan),1);

%% initializing matlab PF implementation
pf = particleFilter(@vdpParticleFilterStateFcn,@vdpMeasurementLikelihoodFcn);
initialize(pf,1000,[2;0],eye(2));
pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';
pf.ResamplingPolicy.MinEffectiveParticleRatio = 2/3;

%% initializing my PF implementation
pf2 = pf_init(@(x,u) vdpParticleFilterStateFcn(x),...
    @meas_likelihood,...
    1e3, [2;0], eye(2), sqrtR^2);

%% simulation
xEst = zeros(size(xTrue));
xEst2 = xEst';
for k=1:size(xTrue,1)
    xEst(k,:) = correct(pf,yMeas(k));
    pf2 = pf_correct(pf2, yMeas(k));
    xEst2(:,k) = pf2.state;
    
    predict(pf);
    pf2 = pf_predict(pf2, 0);
end

%% plotting
figure(1)
clf
hold on
plot(xTrue(:,1),xTrue(:,2),'--','linewidth',2)
plot(xEst(:,1),xEst(:,2),'ro')
plot(xEst2(1,:),xEst2(2,:),'kx','markersize',8)
legend('True','Estimated')

%% removing data
rmpath(fullfile(matlabroot,'examples','control','main')) % remove example data


function [ z ] = meas_fcn( x )
    z = x(1,:);
end
function [ likelihood ] = meas_likelihood( particles,meas,meas_noise )
    [nz,~] = size(meas);
    pred_meas = meas_fcn(particles);
    
    residual = pred_meas-meas;
    meas_error_prod = dot(residual, meas_noise\residual, 1);
    
    likelihood = 1/sqrt((2*pi).^nz*det(meas_noise))*exp(-0.5*meas_error_prod);
end