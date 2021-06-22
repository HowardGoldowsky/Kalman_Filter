
function [Px, Py, Vx, Vy] = ToyBalloonData(P0, V0, mu_wind, sigma_wind, deltaT, numSamples)

% Quick script to simuilate toy ballaistic data with zero air resistance
% and only gravity.

% INPUT:
%   Vo   :      initial velocity at the ground (m/s)
%   Po   :      initial position (m)
%   mu_wind
%   sigma_wind

% OUTPUT:
%   Px  :       x position
%   Py  :       y position
%   Vx  :       x velocity
%   Vy  :       y velocity

% create position based on force from random acceleration
% pos = 0.5 * a * deltaT^2
ax = [P0(1), normrnd(mu_wind,sigma_wind,[1,numSamples])];     
ay = [P0(2), normrnd(mu_wind,sigma_wind,[1,numSamples])];
Px = 0.5 * deltaT * cumsum(ax);
Py = 0.5 * deltaT * cumsum(ay);

% velocity is dx/dt
Vx = [V0(1), diff(Px)];                                      
Vy = [V0(2), diff(Py)];

end % function