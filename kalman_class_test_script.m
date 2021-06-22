
% Test script to run Probabilistic Robotics Homework 1

% Initialize filter and model parameters, create truth data 
[deltaT, A, Q, B, U, P, R, H, X, truthData, P0, V0, SENSOR_WORKING] = kalman_class_FiltINIT_extra_credit();

% Track constructor
myTrack = Track(X, A, Q, B, U, P, R, H);

% Init output
[N, numObs] = size(truthData);
stateEstimate = nan(N,numObs); 

% Run Kalman Filter
for i = 1:numObs
   
%     if (i==6), SENSOR_WORKING=true;end
    [myTrack] = kalmanFilter(myTrack, truthData(:,i), SENSOR_WORKING(i) == 1);
    stateEstimate(:,i) = myTrack.X_bel;
%    Pest(:,:,i) = myTrack.P;
    
end

% Plots

figure; % truth observations and smooth track
plot(stateEstimate(1,:),'b-');
hold on
plot(truthData(1,:),'r-x');
xlabel('Time (s)')
ylabel('X-position')
legend('Best Estimate','Truth Data')

% figure; % Q 1.3: position vs. velocity state
% plot(truthData(1,:),truthData(2,:),'ko-');
% xlabel('Balloon Position (m)')
% ylabel('Balloon Velocity (m/s)')

% truthData
% Pest

%figure; % Q 1.4: position vs. velocity state with uncertainty ellipse
% PlotUncertaintyEllipse(truthData(:,1:2)');
% PlotUncertaintyEllipse(truthData(:,1:3)');
% PlotUncertaintyEllipse(truthData(:,1:4)');
% PlotUncertaintyEllipse(truthData(:,1:5)');

% figure; % truth observations and smooth track EXTRA CREDIT
% time = 1:numObs;
% plot3(time,stateEstimate(1,:),stateEstimate(2,:),'b-');
% hold on
% plot3(time,truthData(1,:),truthData(2,:),'r-x');
% xlabel('Time (s)')
% ylabel('X-position')
% zlabel('Y-position')
% legend('Best Estimate','Truth Data')
% grid on;